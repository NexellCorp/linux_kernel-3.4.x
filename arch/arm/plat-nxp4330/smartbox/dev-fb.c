/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
/*
 * HDMI
 */
#ifdef CONFIG_NEXELL_SOC_DISP_HDMI

static struct nxp_hdmi_plat_data hdmi_data = {
	.preset			= 0,	/* 0 = 1280 * 720p, 1=1920 * 1080p */
	.display_in		= DISP_DEVICE_SYNCGEN0,
};

static struct platform_device hdmi_device = {
	.name	= DEV_NAME_HDMI,
	.id		= -1,
	.dev    = {
		.platform_data	= &hdmi_data,
	},
};
#endif /* HDMI */


/*------------------------------------------------------------------------------
 * Frame Buffer (Primary)
 */
#if defined (CONFIG_FB_NEXELL_PRI)
static struct nxp_fb_plat_data pri_fb_plat_data = {
	.module			= 0,
	.layer			= CFG_DISP_PRI_SCREEN_LAYER,
	.format			= CFG_DISP_PRI_SCREEN_RGB_FORMAT,
	.bgcolor		= CFG_DISP_PRI_BACK_GROUND_COLOR,
	.bitperpixel	= CFG_DISP_PRI_SCREEN_PIXEL_BYTE * 8,
	.x_resol		= CFG_DISP_PRI_RESOL_WIDTH,
	.y_resol		= CFG_DISP_PRI_RESOL_HEIGHT,
#ifdef CONFIG_ANDROID
	.buffers		= 3,
	.skip_pan_vsync	= 1,
#else
	.buffers		= 2,
#endif
	/* display sync for dpi */
	.lcd_with_mm	= 152.4,
	.lcd_height_mm	=  91.44,
};

/* platform device data */
static struct platform_device fb_primary = {
	.name	= DEV_NAME_FB,
	.id		= 0,	/* device channel */
	.dev    = {
		.coherent_dma_mask 	= 0xffffffffUL,	/* for DMA allocate */
		.platform_data		= &pri_fb_plat_data
	},
};
#endif	/* CONFIG_FB_NEXELL_PRI */

/*------------------------------------------------------------------------------
 *	platform devices
 */
static void __init nxp_fb_device_register(void)
{
#if defined(CONFIG_FB_NEXELL_PRI)
	printk("plat: add device frame buffer\n");
	platform_device_register(&fb_primary);
#endif

#if defined(CONFIG_NEXELL_SOC_DISP_HDMI)
	printk("plat: add device hdmi \n");
	platform_device_register(&hdmi_device);
#endif
};
