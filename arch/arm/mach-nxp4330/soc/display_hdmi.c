/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <mach/platform.h>
#include <linux/platform_device.h>

#include <mach/devices.h>
#include <mach/soc.h>
#include "display_4330.h"

#include <nx_hdmi.h>
#include <nx_rstcon.h>
#include <nx_displaytop.h>
#include <nx_disptop_clkgen.h>
#include <nx_ecid.h>
#include <nx_tieoff.h>

#include "regs-hdmi.h"
#include "hdmi-priv.h"

#if (0)
#define DBGOUT(msg...)		{ printk(KERN_INFO msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define DEFAULT_SAMPLE_RATE             48000
#define DEFAULT_BITS_PER_SAMPLE         16
#define DEFAULT_AUDIO_CODEC             HDMI_AUDIO_PCM

#define NXP_HDMIPHY_PRESET_TABLE_SIZE   30

static const u8 hdmiphy_preset74_25[32] = {
    0xd1, 0x1f, 0x10, 0x40, 0x40, 0xf8, 0xc8, 0x81,
    0xe8, 0xba, 0xd8, 0x45, 0xa0, 0xac, 0x80, 0x56,
    0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
    0xa5, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80, 0x10,
};

static const u8 hdmiphy_preset148_5[32] = {
    0xd1, 0x1f, 0x00, 0x40, 0x40, 0xf8, 0xc8, 0x81,
    0xe8, 0xba, 0xd8, 0x45, 0xa0, 0xac, 0x80, 0x66,
    0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x86, 0x54,
    0x4b, 0x25, 0x03, 0x00, 0x00, 0x01, 0x80, 0x10,
};

enum NXP_HDMI_PRESET {
    NXP_HDMI_PRESET_720P = 0,   /* 1280 x 720 */
    NXP_HDMI_PRESET_1080P,      /* 1920 x 1080 */
    NXP_HDMI_PRESET_MAX
};

struct nxp_hdmi_context {
    bool initialized;
    bool streaming;

    u32 audio_codec;
    bool audio_enable;
    u32 audio_channel_count;
    int sample_rate;
    int bits_per_sample;

    u32 aspect;
    int color_range;
    bool is_dvi;
    int vic;

    enum NXP_HDMI_PRESET cur_preset;

    struct mutex mutex;

    u32 v_sync_start;
    u32 h_active_start;
    u32 h_active_end;
    u32 v_sync_hs_start_end0;
    u32 v_sync_hs_start_end1;

    u32 source_dpc_module_num;
    u32 width;
    u32 height;
    int source_device;
};

static struct nxp_hdmi_context *_context = NULL;

static int _hdmiphy_reg_set(const u8 *data, size_t size)
{
    int i;
    u32 reg_addr;

    NX_HDMI_SetReg(0, HDMI_PHY_Reg7C, (0<<7)); NX_HDMI_SetReg(0, HDMI_PHY_Reg7C, (0<<7));
    NX_HDMI_SetReg(0, HDMI_PHY_Reg04, (0<<4)); NX_HDMI_SetReg(0, HDMI_PHY_Reg04, (0<<4));
    NX_HDMI_SetReg(0, HDMI_PHY_Reg24, (1<<7)); NX_HDMI_SetReg(0, HDMI_PHY_Reg24, (1<<7));

    for (i = 0, reg_addr = HDMI_PHY_Reg04; i < size; i++, reg_addr += 4)
        NX_HDMI_SetReg(0, reg_addr, data[i]); NX_HDMI_SetReg(0, reg_addr, data[i]);

    NX_HDMI_SetReg(0, HDMI_PHY_Reg7C, 0x80); NX_HDMI_SetReg(0, HDMI_PHY_Reg7C, 0x80);
    NX_HDMI_SetReg(0, HDMI_PHY_Reg7C, (1<<7)); NX_HDMI_SetReg(0, HDMI_PHY_Reg7C, (1<<7));
    return 0;
}

static int _hdmi_phy_enable(struct nxp_hdmi_context *me, int enable)
{
    int ret;

    if (enable) {
        const u8 *table;
        switch (me->cur_preset) {
        case NXP_HDMI_PRESET_720P:
            table = hdmiphy_preset74_25;
            break;
        case NXP_HDMI_PRESET_1080P:
            table = hdmiphy_preset148_5;
            break;
        default:
            printk(KERN_ERR "%s: invalid preset %d\n", __func__, me->cur_preset);
            return -EINVAL;
        }

        ret = _hdmiphy_reg_set(table, NXP_HDMIPHY_PRESET_TABLE_SIZE);
        if (ret < 0) {
            printk(KERN_ERR "%s: failed to _hdmiphy_reg_set()\n", __func__);
            return ret;
        }
    }

    return 0;
}

static inline bool _wait_for_ecid_ready(void)
{
    int retry_count = 100;
    bool is_key_ready = false;

    NX_ECID_SetBaseAddress((U32)IO_ADDRESS(NX_ECID_GetPhysicalAddress()));

    do {
        is_key_ready = NX_ECID_GetKeyReady();
        if (is_key_ready) break;
        msleep(1);
        retry_count--;
    } while (retry_count);

    return is_key_ready;
}

static inline void _hdmi_reset(void)
{
    NX_RSTCON_SetnRST(NX_HDMI_GetResetNumber(0, i_nRST_VIDEO), RSTCON_nDISABLE);
    NX_RSTCON_SetnRST(NX_HDMI_GetResetNumber(0, i_nRST_SPDIF), RSTCON_nDISABLE);
    NX_RSTCON_SetnRST(NX_HDMI_GetResetNumber(0, i_nRST_TMDS), RSTCON_nDISABLE);

    NX_RSTCON_SetnRST(NX_HDMI_GetResetNumber(0, i_nRST_VIDEO), RSTCON_nENABLE);
    NX_RSTCON_SetnRST(NX_HDMI_GetResetNumber(0, i_nRST_SPDIF), RSTCON_nENABLE);
    NX_RSTCON_SetnRST(NX_HDMI_GetResetNumber(0, i_nRST_TMDS), RSTCON_nENABLE);
}

static inline bool _wait_for_hdmiphy_ready(void)
{
    int retry_count = 500;
    do {
        u32 regval = NX_HDMI_GetReg(0, HDMI_LINK_PHY_STATUS_0);
        if (regval & 0x01) {
            printk("HDMI PHY Ready!!!\n");
            return true;
        }
        mdelay(10);
        retry_count--;
    } while (retry_count);

    return false;
}

static inline int _get_vsync_info(int preset, int device,
				struct disp_vsync_info *vsync, struct disp_syncgen_param *par)
{
	nxp_soc_disp_device_get_param(device, (void*)par);

    switch (preset) {
    case NXP_HDMI_PRESET_720P:
        /* 720p: 1280x720 */
        vsync->h_active_len = 1280;
        vsync->h_sync_width = 40;
        vsync->h_back_porch = 220;
        vsync->h_front_porch = 110;
        vsync->h_sync_invert = 0;
        vsync->v_active_len = 720;
        vsync->v_sync_width = 5;
        vsync->v_back_porch = 20;
        vsync->v_front_porch = 5;
        vsync->v_sync_invert = 0;
        vsync->pixel_clock_hz = 74250000;
        break;

    case NXP_HDMI_PRESET_1080P:
        /* 1080p: 1920x1080 */
        vsync->h_active_len = 1920;
        vsync->h_sync_width = 44;
        vsync->h_back_porch = 148;
        vsync->h_front_porch = 88;
        vsync->h_sync_invert = 0;
        vsync->v_active_len = 1080;
        vsync->v_sync_width = 5;
        vsync->v_back_porch = 36;
        vsync->v_front_porch = 4;
        vsync->v_sync_invert = 0;
        vsync->pixel_clock_hz = 148500000;
        break;

    default:
        printk(KERN_ERR "%s: invalid preset value 0x%x\n", __func__, preset);
        return -EINVAL;
    }

    vsync->clk_src_lv0 = 4;
    vsync->clk_div_lv0 = 1;
    vsync->clk_src_lv1 = 7;
    vsync->clk_div_lv1 = 1;

	par->out_format	= OUTPUTFORMAT_RGB888;
	par->delay_mask = (DISP_SYNCGEN_DELAY_RGB_PVD | DISP_SYNCGEN_DELAY_HSYNC_CP1 |
					   DISP_SYNCGEN_DELAY_VSYNC_FRAM | DISP_SYNCGEN_DELAY_DE_CP);
	par->d_rgb_pvd = 0;
	par->d_hsync_cp1 = 0;
	par->d_vsync_fram = 0;
	par->d_de_cp2 = 7;

	//	HFP + HSW + HBP + AVWidth-VSCLRPIXEL- 1;
	par->vs_start_offset = (vsync->h_front_porch + vsync->h_sync_width +
						vsync->h_back_porch + vsync->h_active_len - 1);
	par->vs_end_offset = 0;
	// HFP + HSW + HBP + AVWidth-EVENVSCLRPIXEL- 1
	par->ev_start_offset = (vsync->h_front_porch + vsync->h_sync_width +
						vsync->h_back_porch + vsync->h_active_len - 1);
	par->ev_end_offset = 0;

    return 0;
}

static int _set_remote_sync(struct nxp_hdmi_context *me)
{
    int ret;
    struct disp_vsync_info vsync;
    struct disp_syncgen_param param;
    int source_device = me->source_device;

    switch (source_device) {
    case DISP_DEVICE_SYNCGEN0:
        NX_DISPLAYTOP_SetHDMIMUX(CTRUE, PrimaryMLC);
        me->source_dpc_module_num = 0;
        break;
    case DISP_DEVICE_SYNCGEN1:
        NX_DISPLAYTOP_SetHDMIMUX(CTRUE, SecondaryMLC);
        me->source_dpc_module_num = 1;
        break;
    case DISP_DEVICE_RESCONV:
        NX_DISPLAYTOP_SetHDMIMUX(CTRUE, ResolutionConv);
        break;
    default:
        printk(KERN_ERR "%s: invalid source device %d\n", __func__, source_device);
        return -EINVAL;
    }

    ret = _get_vsync_info(me->cur_preset, source_device, &vsync, &param);
    if (ret) {
        printk(KERN_ERR "%s: failed to _get_vsync_info()\n", __func__);
        return ret;
    }

    /* vsync.interlace_scan = me->cur_conf->mbus_fmt.field == V4L2_FIELD_INTERLACED; */
    vsync.interlace = 0;

    ret = nxp_soc_disp_device_set_param(source_device, (void*)&param);
    if (ret) {
        pr_err("%s: failed to display parameter....\n", __func__);
        return ret;
    }

#if (0)
 	ret = nxp_soc_disp_device_connect_to(DISP_DEVICE_HDMI, source_device, &vsync);
    if (ret) {
        printk(KERN_ERR "%s: failed to connect to source\n", __func__);
        return ret;
    }
	ret = nxp_soc_disp_device_enable(source_device, 1);
#endif
	return ret;
}

static void _set_hdmi_clkgen(void)
{
    NX_DISPTOP_CLKGEN_SetBaseAddress(ToMIPI_CLKGEN,
            (U32)IO_ADDRESS(NX_DISPTOP_CLKGEN_GetPhysicalAddress(ToMIPI_CLKGEN)));
    NX_DISPTOP_CLKGEN_SetClockDivisorEnable(ToMIPI_CLKGEN, CFALSE);
    NX_DISPTOP_CLKGEN_SetClockPClkMode(ToMIPI_CLKGEN, NX_PCLKMODE_ALWAYS);
    NX_DISPTOP_CLKGEN_SetClockSource(ToMIPI_CLKGEN, HDMI_SPDIF_CLKOUT, 2); // pll2
    NX_DISPTOP_CLKGEN_SetClockDivisor(ToMIPI_CLKGEN, HDMI_SPDIF_CLKOUT, 2);
    NX_DISPTOP_CLKGEN_SetClockSource(ToMIPI_CLKGEN, 1, 7);
    NX_DISPTOP_CLKGEN_SetClockDivisorEnable(ToMIPI_CLKGEN, CTRUE);
}

static void _set_audio_clkgen(void)
{
    NX_DISPTOP_CLKGEN_SetBaseAddress(ToMIPI_CLKGEN,
            (U32)IO_ADDRESS(NX_DISPTOP_CLKGEN_GetPhysicalAddress(ToMIPI_CLKGEN)));
    NX_DISPTOP_CLKGEN_SetClockPClkMode(ToMIPI_CLKGEN, NX_PCLKMODE_ALWAYS);
    NX_DISPTOP_CLKGEN_SetClockDivisorEnable(ToMIPI_CLKGEN, CTRUE);
}

static void _hdmi_sync_init(void)
{
    NX_DISPLAYTOP_HDMI_SetVSyncHSStartEnd(0, 0);
    NX_DISPLAYTOP_HDMI_SetVSyncStart(0);
    NX_DISPLAYTOP_HDMI_SetHActiveStart(0);
    NX_DISPLAYTOP_HDMI_SetHActiveEnd(0);
}

static int _hdmi_config(struct nxp_hdmi_context *me)
{
    u32 width;
    u32 height;
    u32 hfp;
    u32 hsw;
    u32 hbp;
    u32 vfp;
    u32 vsw;
    u32 vbp;

    u32 h_blank;
    u32 v_blank;
    u32 v_actline;
    u32 v2_blank;
    u32 v_line;
    u32 h_line;
    u32 h_sync_start;
    u32 h_sync_end;
    u32 v_sync_line_bef_1;
    u32 v_sync_line_bef_2;

    u32 fixed_ffff = 0xffff;

    switch (me->cur_preset) {
    case NXP_HDMI_PRESET_720P:
        printk("%s: 720p\n", __func__);
        width = 1280;
        height = 720;
        hfp = 110;
        hsw = 40;
        hbp = 220;
        vfp = 5;
        vsw = 5;
        vbp = 20;
        break;
    case NXP_HDMI_PRESET_1080P:
        printk("%s: 1080p\n", __func__);
        width = 1920;
        height = 1080;
        hfp = 88;
        hsw = 44;
        hbp = 148;
        vfp = 4;
        vsw = 5;
        vbp = 36;
        break;
    default:
        printk(KERN_ERR "%s: invalid preset %d\n", __func__, me->cur_preset);
        return -EINVAL;
    }

    me->width = width;
    me->height = height;

    /**
     * calculate sync variables
     */
    h_blank = hfp + hsw + hbp;
    v_blank = vfp + vsw + vbp;
    v_actline = height;
    v2_blank = height + vfp + vsw + vbp;
    v_line = height + vfp + vsw + vbp; /* total v */
    h_line = width + hfp + hsw + hbp;  /* total h */
    h_sync_start = hfp;
    h_sync_end = hfp + hsw;
    v_sync_line_bef_1 = vfp;
    v_sync_line_bef_2 = vfp + vsw;

    /* for debugging */
    /* printk("h_blank %d, v_blank %d, v_actline %d, v2_blank %d, v_line %d, h_line %d, h_sync_start %d, h_sync_end %d, v_sync_line_bef_1 %d, v_sync_line_bef_2 %d\n", */
    /*         h_blank, v_blank, v_actline, v2_blank, v_line, h_line, h_sync_start, h_sync_end, v_sync_line_bef_1, v_sync_line_bef_2); */

    /* no blue screen mode, encoding order as it is */
    NX_HDMI_SetReg(0, HDMI_LINK_HDMI_CON_0, (0<<5)|(1<<4));

    /* set HDMI_LINK_BLUE_SCREEN_* to 0x0 */
    NX_HDMI_SetReg(0, HDMI_LINK_BLUE_SCREEN_R_0, 0x5555);
    NX_HDMI_SetReg(0, HDMI_LINK_BLUE_SCREEN_R_1, 0x5555);
    NX_HDMI_SetReg(0, HDMI_LINK_BLUE_SCREEN_G_0, 0x5555);
    NX_HDMI_SetReg(0, HDMI_LINK_BLUE_SCREEN_G_1, 0x5555);
    NX_HDMI_SetReg(0, HDMI_LINK_BLUE_SCREEN_B_0, 0x5555);
    NX_HDMI_SetReg(0, HDMI_LINK_BLUE_SCREEN_B_1, 0x5555);

    /* set HDMI_CON_1 to 0x0 */
    NX_HDMI_SetReg(0, HDMI_LINK_HDMI_CON_1, 0x0);
    NX_HDMI_SetReg(0, HDMI_LINK_HDMI_CON_2, 0x0);

    /* set interrupt : enable hpd_plug, hpd_unplug */
    NX_HDMI_SetReg(0, HDMI_LINK_INTC_CON_0, (1<<6)|(1<<3)|(1<<2));

    /* set STATUS_EN to 0x17 */
    NX_HDMI_SetReg(0, HDMI_LINK_STATUS_EN, 0x17);

    /* TODO set HDP to 0x0 : later check hpd */
    NX_HDMI_SetReg(0, HDMI_LINK_HPD, 0x0);

    /* set MODE_SEL to 0x02 */
    NX_HDMI_SetReg(0, HDMI_LINK_MODE_SEL, 0x2);

    /* set H_BLANK_*, V1_BLANK_*, V2_BLANK_*, V_LINE_*, H_LINE_*, H_SYNC_START_*, H_SYNC_END_ *
     * V_SYNC_LINE_BEF_1_*, V_SYNC_LINE_BEF_2_*
     */
    NX_HDMI_SetReg(0, HDMI_LINK_H_BLANK_0, h_blank%256);
    NX_HDMI_SetReg(0, HDMI_LINK_H_BLANK_1, h_blank>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V1_BLANK_0, v_blank%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V1_BLANK_1, v_blank>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V2_BLANK_0, v2_blank%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V2_BLANK_1, v2_blank>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_LINE_0, v_line%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_LINE_1, v_line>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_H_LINE_0, h_line%256);
    NX_HDMI_SetReg(0, HDMI_LINK_H_LINE_1, h_line>>8);

    if (width == 1280) {
        NX_HDMI_SetReg(0, HDMI_LINK_HSYNC_POL, 0x1);
        NX_HDMI_SetReg(0, HDMI_LINK_VSYNC_POL, 0x1);
    } else {
        NX_HDMI_SetReg(0, HDMI_LINK_HSYNC_POL, 0x0);
        NX_HDMI_SetReg(0, HDMI_LINK_VSYNC_POL, 0x0);
    }

    NX_HDMI_SetReg(0, HDMI_LINK_INT_PRO_MODE, 0x0);

    NX_HDMI_SetReg(0, HDMI_LINK_H_SYNC_START_0, (h_sync_start%256)-2);
    NX_HDMI_SetReg(0, HDMI_LINK_H_SYNC_START_1, h_sync_start>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_H_SYNC_END_0, (h_sync_end%256)-2);
    NX_HDMI_SetReg(0, HDMI_LINK_H_SYNC_END_1, h_sync_end>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_BEF_1_0, v_sync_line_bef_1%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_BEF_1_1, v_sync_line_bef_1>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_BEF_2_0, v_sync_line_bef_2%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_BEF_2_1, v_sync_line_bef_2>>8);

    /* Set V_SYNC_LINE_AFT*, V_SYNC_LINE_AFT_PXL*, VACT_SPACE* */
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_1_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_1_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_2_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_2_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_3_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_3_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_4_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_4_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_5_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_5_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_6_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_6_1, fixed_ffff>>8);

    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_1_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_1_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_2_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_2_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_3_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_3_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_4_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_4_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_5_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_5_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_6_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_V_SYNC_LINE_AFT_PXL_6_1, fixed_ffff>>8);

    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE1_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE1_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE2_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE2_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE3_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE3_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE4_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE4_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE5_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE5_1, fixed_ffff>>8);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE6_0, fixed_ffff%256);
    NX_HDMI_SetReg(0, HDMI_LINK_VACT_SPACE6_1, fixed_ffff>>8);

    NX_HDMI_SetReg(0, HDMI_LINK_CSC_MUX, 0x0);
    NX_HDMI_SetReg(0, HDMI_LINK_SYNC_GEN_MUX, 0x0);

    NX_HDMI_SetReg(0, HDMI_LINK_SEND_START_0, 0xfd);
    NX_HDMI_SetReg(0, HDMI_LINK_SEND_START_1, 0x01);
    NX_HDMI_SetReg(0, HDMI_LINK_SEND_END_0, 0x0d);
    NX_HDMI_SetReg(0, HDMI_LINK_SEND_END_1, 0x3a);
    NX_HDMI_SetReg(0, HDMI_LINK_SEND_END_2, 0x08);

    /* Set DC_CONTROL to 0x00 */
    NX_HDMI_SetReg(0, HDMI_LINK_DC_CONTROL, 0x0);

    /* Set VIDEO_PATTERN_GEN to 0x00 */
#if (1)
    NX_HDMI_SetReg(0, HDMI_LINK_VIDEO_PATTERN_GEN, 0x0);
#else
    NX_HDMI_SetReg(0, HDMI_LINK_VIDEO_PATTERN_GEN, 0x1);
#endif

    NX_HDMI_SetReg(0, HDMI_LINK_GCP_CON, 0x0a);

    /* Set HDMI Sync Control parameters */
    me->v_sync_start = vsw + vbp + height - 1;
    me->h_active_start = hsw + hbp;
    me->h_active_end = width + hsw + hbp;
    me->v_sync_hs_start_end0 = hsw + hbp + 1;
    me->v_sync_hs_start_end1 = hsw + hbp + 2;

    return 0;
}

static void _hdmi_start(struct nxp_hdmi_context *me)
{
    u32 regval = NX_HDMI_GetReg(0, HDMI_LINK_HDMI_CON_0);
    regval |= 0x01;
    NX_HDMI_SetReg(0, HDMI_LINK_HDMI_CON_0, regval);

    NX_DISPLAYTOP_HDMI_SetVSyncStart(me->v_sync_start);
    NX_DISPLAYTOP_HDMI_SetHActiveStart(me->h_active_start);
    NX_DISPLAYTOP_HDMI_SetHActiveEnd(me->h_active_end);
    NX_DISPLAYTOP_HDMI_SetVSyncHSStartEnd(me->v_sync_hs_start_end0, me->v_sync_hs_start_end1);
}

static void _hdmi_reg_infoframe(struct nxp_hdmi_context *me,
        struct hdmi_infoframe *infoframe,
        const struct hdmi_3d_info *info_3d)
{
    u32 hdr_sum;
    u8 chksum;
    u32 aspect_ratio;
    u32 vic;

    pr_debug("%s: infoframe type = 0x%x\n", __func__, infoframe->type);

    if (me->is_dvi) {
        hdmi_writeb(HDMI_VSI_CON, HDMI_VSI_CON_DO_NOT_TRANSMIT);
        hdmi_writeb(HDMI_AVI_CON, HDMI_AVI_CON_DO_NOT_TRANSMIT);
        hdmi_write(HDMI_AUI_CON, HDMI_AUI_CON_NO_TRAN);
        return;
    }

    switch (infoframe->type) {
    case HDMI_PACKET_TYPE_VSI:
        hdmi_writeb(HDMI_VSI_CON, HDMI_VSI_CON_EVERY_VSYNC);
        hdmi_writeb(HDMI_VSI_HEADER0, infoframe->type);
        hdmi_writeb(HDMI_VSI_HEADER1, infoframe->ver);
        /* 0x000C03 : 24bit IEEE Registration Identifier */
        hdmi_writeb(HDMI_VSI_DATA(1), 0x03);
        hdmi_writeb(HDMI_VSI_DATA(2), 0x0c);
        hdmi_writeb(HDMI_VSI_DATA(3), 0x00);
        hdmi_writeb(HDMI_VSI_DATA(4),
                HDMI_VSI_DATA04_VIDEO_FORMAT(info_3d->is_3d));
        hdmi_writeb(HDMI_VSI_DATA(5),
                HDMI_VSI_DATA05_3D_STRUCTURE(info_3d->fmt_3d));
        if (info_3d->fmt_3d == HDMI_3D_FORMAT_SB_HALF) {
            infoframe->len += 1;
            hdmi_writeb(HDMI_VSI_DATA(6),
                    (u8)HDMI_VSI_DATA06_3D_EXT_DATA(HDMI_H_SUB_SAMPLE));
        }
        hdmi_writeb(HDMI_VSI_HEADER2, infoframe->len);
        hdr_sum = infoframe->type + infoframe->ver + infoframe->len;
        chksum = hdmi_chksum(HDMI_VSI_DATA(1), infoframe->len, hdr_sum);
        pr_debug("%s: VSI checksum = 0x%x\n", __func__, chksum);
        hdmi_writeb(HDMI_VSI_DATA(0), chksum);
        break;

    case HDMI_PACKET_TYPE_AVI:
        hdmi_writeb(HDMI_AVI_CON, HDMI_AVI_CON_EVERY_VSYNC);
        hdmi_writeb(HDMI_AVI_HEADER0, infoframe->type);
        hdmi_writeb(HDMI_AVI_HEADER1, infoframe->ver);
        hdmi_writeb(HDMI_AVI_HEADER2, infoframe->len);
        hdr_sum = infoframe->type + infoframe->ver + infoframe->len;
        hdmi_writeb(HDMI_AVI_BYTE(1), HDMI_OUTPUT_RGB888 << 5 |
                AVI_ACTIVE_FORMAT_VALID | AVI_UNDERSCAN);
        aspect_ratio = AVI_PIC_ASPECT_RATIO_16_9;
        vic = me->vic;

        hdmi_writeb(HDMI_AVI_BYTE(2), aspect_ratio |
                AVI_SAME_AS_PIC_ASPECT_RATIO | AVI_ITU709);
        if (me->color_range == 0 || me->color_range == 2)
            hdmi_writeb(HDMI_AVI_BYTE(3), AVI_FULL_RANGE);
        else
            hdmi_writeb(HDMI_AVI_BYTE(3), AVI_LIMITED_RANGE);
        pr_debug("%s: VIC code = %d\n", __func__, vic);
        hdmi_writeb(HDMI_AVI_BYTE(4), vic);
        chksum = hdmi_chksum(HDMI_AVI_BYTE(1), infoframe->len, hdr_sum);
        pr_debug("%s: AVI checksum = 0x%x\n", __func__, chksum);
        hdmi_writeb(HDMI_AVI_CHECK_SUM, chksum);
        break;

    case HDMI_PACKET_TYPE_AUI:
        hdmi_write(HDMI_AUI_CON, HDMI_AUI_CON_TRANS_EVERY_VSYNC);
        hdmi_writeb(HDMI_AUI_HEADER0, infoframe->type);
        hdmi_writeb(HDMI_AUI_HEADER1, infoframe->ver);
        hdmi_writeb(HDMI_AUI_HEADER2, infoframe->len);
        hdr_sum = infoframe->type + infoframe->ver + infoframe->len;
        /* speaker placement */
        if (me->audio_channel_count == 6)
            hdmi_writeb(HDMI_AUI_BYTE(4), 0x0b);
        else if (me->audio_channel_count == 8)
            hdmi_writeb(HDMI_AUI_BYTE(4), 0x13);
        else
            hdmi_writeb(HDMI_AUI_BYTE(4), 0x00);
        chksum = hdmi_chksum(HDMI_AUI_BYTE(1), infoframe->len, hdr_sum);
        pr_debug("%s: AUI checksum = 0x%x\n", __func__, chksum);
        hdmi_writeb(HDMI_AUI_CHECK_SUM, chksum);
        break;

    default:
        pr_err("%s: unknown type(0x%x)\n", __func__, infoframe->type);
        break;
    }
}

static int _hdmi_set_infoframe(struct nxp_hdmi_context *me)
{
    struct hdmi_infoframe infoframe;
    struct hdmi_3d_info info_3d;

    info_3d.is_3d = 0;

    hdmi_stop_vsi();

    infoframe.type = HDMI_PACKET_TYPE_AVI;
    infoframe.ver  = HDMI_AVI_VERSION;
    infoframe.len  = HDMI_AVI_LENGTH;
    _hdmi_reg_infoframe(me, &infoframe, &info_3d);

    if (me->audio_enable) {
        infoframe.type = HDMI_PACKET_TYPE_AUI;
        infoframe.ver  = HDMI_AUI_VERSION;
        infoframe.len  = HDMI_AUI_LENGTH;
        _hdmi_reg_infoframe(me, &infoframe, &info_3d);
    }

    return 0;
}

static void _hdmi_set_packets(struct nxp_hdmi_context *me)
{
    hdmi_set_acr(me->sample_rate, me->is_dvi);
}

void nxp_soc_disp_hdmi_initialize(void)
{
    struct nxp_hdmi_context *me = _context;

    if (!me) {
        printk(KERN_ERR "hdmi soc driver not probed!!!\n");
        return;
    }

    if (!me->initialized) {
        /**
         * [SEQ 1] release the reset of DisplayTop.i_Top_nRst)
         */
#if 0
        NX_RSTCON_SetnRST(NX_DISPLAYTOP_GetResetNumber(), RSTCON_nDISABLE);
        NX_RSTCON_SetnRST(NX_DISPLAYTOP_GetResetNumber(), RSTCON_nENABLE);
#endif

        /**
         * [SEQ 2] set the HDMI CLKGEN's PCLKMODE to always enabled
         */
        NX_DISPTOP_CLKGEN_SetBaseAddress(HDMI_CLKGEN,
                (U32)IO_ADDRESS(NX_DISPTOP_CLKGEN_GetPhysicalAddress(HDMI_CLKGEN)));
        NX_DISPTOP_CLKGEN_SetClockPClkMode(HDMI_CLKGEN, NX_PCLKMODE_ALWAYS);

        NX_HDMI_SetBaseAddress(0, (U32)IO_ADDRESS(NX_HDMI_GetPhysicalAddress(0)));
        NX_HDMI_Initialize();

        /**
         * [SEQ 3] set the 0xC001100C[0] to 1
         */
        /* NX_TIEOFF_Initialize(); */
        NX_TIEOFF_SetBaseAddress((U32)IO_ADDRESS(NX_TIEOFF_GetPhysicalAddress()));
        NX_TIEOFF_Set(TIEOFFINDEX_OF_DISPLAYTOP0_i_HDMI_PHY_REFCLK_SEL, 1);

        /**
         * [SEQ 4] release the resets of HDMI.i_PHY_nRST and HDMI.i_nRST
         */
        NX_RSTCON_SetnRST(NX_HDMI_GetResetNumber(0, i_nRST_PHY), RSTCON_nDISABLE);
        NX_RSTCON_SetnRST(NX_HDMI_GetResetNumber(0, i_nRST), RSTCON_nDISABLE);

        NX_RSTCON_SetnRST(NX_HDMI_GetResetNumber(0, i_nRST_PHY), RSTCON_nENABLE);
        NX_RSTCON_SetnRST(NX_HDMI_GetResetNumber(0, i_nRST), RSTCON_nENABLE);

        me->initialized = true;

        /**
         * Next sequence start in nxp_soc_disp_hdmi_enable()
         */
    }
}

/* int nxp_soc_disp_hdmi_streamon(struct nxp_hdmi_context *me) */
int nxp_soc_disp_hdmi_enable(int enable)
{
    int ret;
    struct nxp_hdmi_context *me = _context;

    if (!me) {
        printk(KERN_ERR "hdmi soc driver not probed!!!\n");
        return -ENODEV;
    }

    if (enable && !me->streaming) {
        /**
         * [SEQ 5] set up the HDMI PHY to specific video clock.
         */
        ret = _hdmi_phy_enable(me, 1);
        if (ret < 0) {
            printk(KERN_ERR "%s: _hdmi_phy_enable() failed\n", __func__);
            return ret;
        }

        /**
         * [SEQ 6] I2S (or SPDIFTX) configuration for the source audio data
         * this is done in another user app  - ex> Android Audio HAL
         */

        /**
         * [SEQ 7] Wait for ECID ready
         */
        if(false == _wait_for_ecid_ready()) {
            printk(KERN_ERR "%s: failed to wait for ecid ready\n", __func__);
            _hdmi_phy_enable(me, 0);
            return -EIO;
        }

        /**
         * [SEQ 8] release the resets of HDMI.i_VIDEO_nRST and HDMI.i_SPDIF_nRST and HDMI.i_TMDS_nRST
         */
        _hdmi_reset();

        /**
         * [SEQ 9] Wait for HDMI PHY ready (wait until 0xC0200020.[0], 1)
         */
        if (false == _wait_for_hdmiphy_ready()) {
            printk(KERN_ERR "%s: failed to wait for hdmiphy ready\n", __func__);
            _hdmi_phy_enable(me, 0);
            return -EIO;
        }

        /**
         * [SEC 10] Set the DPC CLKGEN’s Source Clock to HDMI_CLK & Set Sync Parameter
         */
        _set_hdmi_clkgen(); /* set hdmi link clk to clkgen  vs default is hdmi phy clk */
        _set_audio_clkgen();
        _hdmi_sync_init();

        /**
         * [SEQ 11] Set up the HDMI Converter parameters
         */
        _hdmi_config(me);
        _set_remote_sync(me);

        _hdmi_set_infoframe(me);
        _hdmi_set_packets(me);

        hdmi_audio_spdif_init(me->audio_codec, me->bits_per_sample);

        if (me->audio_enable)
            hdmi_audio_enable(true);

        /* hdmi_set_dvi_mode(me->is_dvi); */

        _hdmi_start(me);

        mdelay(5);
        me->streaming = true;
    } else {
        me->streaming = false;
        _hdmi_phy_enable(me, 0);
    }

    return 0;
}

int nxp_soc_disp_hdmi_set_preset(int preset)
{
    struct nxp_hdmi_context *me = _context;

    if (!me) {
        printk(KERN_ERR "hdmi soc driver not probed!!!\n");
        return -ENODEV;
    }

    if (preset >= NXP_HDMI_PRESET_MAX) {
        printk(KERN_ERR "%s: invalid preset %d\n", __func__, preset);
        return -EINVAL;
    }

    return 0;
}

EXPORT_SYMBOL(nxp_soc_disp_hdmi_initialize);
EXPORT_SYMBOL(nxp_soc_disp_hdmi_enable);
EXPORT_SYMBOL(nxp_soc_disp_hdmi_set_preset);


static int  nxp_hdmi_enable(struct disp_process_dev *dev, int enable)
{
	DBGOUT("%s %s, %s\n", __func__, dev_to_str(dev->dev_id), enable?"ON":"OFF");
	return nxp_soc_disp_hdmi_enable(enable);
}

static struct disp_process_ops hdmi_ops = {
	.enable 	= nxp_hdmi_enable,
};

static int hdmi_probe(struct platform_device *pdev)
{
	struct nxp_hdmi_plat_data *plat = pdev->dev.platform_data;
    struct nxp_hdmi_context *ctx = NULL;
    struct disp_vsync_info vsync;
    struct disp_syncgen_param param;

	RET_ASSERT_VAL(plat, -EINVAL);
	RET_ASSERT_VAL(plat->display_in == DISP_DEVICE_SYNCGEN0 ||
				   plat->display_in == DISP_DEVICE_SYNCGEN1 ||
				   plat->display_in == DISP_DEVICE_RESCONV, -EINVAL);

    ctx = (struct nxp_hdmi_context *)kzalloc(sizeof(struct nxp_hdmi_context), GFP_KERNEL);
    if (!ctx) {
        printk(KERN_ERR "failed to allocation nxp_hdmi_context\n");
        return -ENOMEM;
    }

    _context = ctx;
    _context->cur_preset = plat->preset;
    _context->source_device = plat->display_in;

    mutex_init(&_context->mutex);

    /* audio */
    _context->audio_enable = true;
    _context->audio_channel_count = 2;
    _context->sample_rate = DEFAULT_SAMPLE_RATE;
    _context->color_range = 3;
    _context->bits_per_sample = DEFAULT_BITS_PER_SAMPLE;
    _context->audio_codec = DEFAULT_AUDIO_CODEC;
    _context->aspect = HDMI_ASPECT_RATIO_16_9;

    hdmi_set_base((void *)IO_ADDRESS(NX_HDMI_GetPhysicalAddress(0)));

	_get_vsync_info(_context->cur_preset, _context->source_device, &vsync, &param);

	nxp_soc_disp_hdmi_initialize();
	nxp_soc_disp_hdmi_set_preset(_context->cur_preset);

    if (_context->cur_preset == NXP_HDMI_PRESET_720P) {
        _context->vic = 4;
    } else if (_context->cur_preset == NXP_HDMI_PRESET_1080P) {
        _context->vic = 16;
    }

	nxp_soc_disp_device_set_param(plat->display_in, &param);
	nxp_soc_disp_register_proc_ops(DISP_DEVICE_HDMI, &hdmi_ops);
	nxp_soc_disp_device_connect_to(DISP_DEVICE_HDMI, _context->source_device, &vsync);
	return 0;
}

static struct platform_driver hdmi_driver = {
	.driver	= {
	.name	= DEV_NAME_HDMI,
	.owner	= THIS_MODULE,
	},
	.probe	= hdmi_probe,
};
module_platform_driver(hdmi_driver);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Display HDMI driver for the Nexell");
MODULE_LICENSE("GPL");
