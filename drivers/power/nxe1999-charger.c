/*
 * nxe2000_charger.c - Power supply consumer driver for the NXE2000
 *
 *  Copyright (C) 2013 Nexell Co,.Ltd.
 *  Bongkwan Kook <kook@nexell.co.kr>
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

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/usb/otg.h>

/* nexell soc headers */
#include <mach/platform.h>
//#include <mach/devices.h>
#include <mach/soc.h>

#include <nxe1999.h>
#include <nxe2000-private.h>

/*
 * Debug
 */
#if (0)
#define DBGOUT(msg...)		{ printk("nxe2000-charger: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif


#define CFG_NXE2000_DEVEL_MODE          (0)
#define CFG_NXE2000_BAT_CHG_SUPPORT     (1)
#define CFG_NXE2000_ADP_ONLY_MODE       (0)     /* for DEBUG */
#define CFG_NXE2000_CHG_PROG_MODE       (0)
#define CFG_NXE2000_SUSPEND_POWER_OFF   (0)

#define NXE2000_USED_TIMER_THREAD
#define NXE2000_RDSTATE_ONLY

#define NXE2000_TIMER_THREAD_PERIODIC	(10000)	/* milisecond		*/


#if (CFG_NXE2000_ADP_ONLY_MODE == 1)
#define NXE2000_CHG_PRIO		(0)		/* 1:VUSB, 0:VADP	*/
#else
#define NXE2000_CHG_PRIO		(1)		/* 1:VUSB, 0:VADP	*/
#endif

#if (NXE2000_CHG_PRIO == 1)
#define NXE2000_CHG_BITS    ( (0x1 << NXE2000_POS_CHGCTL1_CHGP)         \
                            | (0x1 << NXE2000_POS_CHGCTL1_VUSBCHGEN) )
#else
#define NXE2000_CHG_BITS    ( (0x0 << NXE2000_POS_CHGCTL1_CHGP)         \
                            | (0x1 << NXE2000_POS_CHGCTL1_VADPCHGEN) )
#endif

#define NXE2000_CHG_MASK    ( (0x1 << NXE2000_POS_CHGCTL1_CHGP)         \
                            | (0x1 << NXE2000_POS_CHGCTL1_VUSBCHGEN)    \
                            | (0x1 << NXE2000_POS_CHGCTL1_VADPCHGEN)    \
                            | (0x1 << NXE2000_POS_CHGCTL1_OTG_BOOST_EN) \
                            | (0x1 << NXE2000_POS_CHGCTL1_SUSPEND) )


#if (CFG_NXE2000_BAT_CHG_SUPPORT == 1)
#include <nxe1999_battery_init.h>
#endif
static int nxe2000_get_bat_voltage(struct i2c_client *i2c, union power_supply_propval *val);


#if defined(CONFIG_USB_DWCOTG)
extern void otg_phy_init(void);
extern void otg_phy_off(void);
extern void otg_clk_enable(void);
extern void otg_clk_disable(void);
extern void nxp4330_otg_set_usb_state(bool connected);
#endif

struct nxe2000_power {
	struct device				*dev;
	struct nxe2000_dev			*iodev;
	struct nxe2000_pdata		platdata;

    struct mutex                lock;

	struct power_supply			wall;
	struct power_supply			usb;
	struct power_supply			battery;

#if defined(CONFIG_USB_CONNECT_NXP_DRV)
	struct usb_phy				*otg_phy;
	struct notifier_block		otg_nb;
	struct work_struct			otg_work;

	unsigned long				event;
	unsigned					max_power;
#endif

#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
	struct delayed_work	        suspend_on_work;
	struct workqueue_struct    *suspend_on_wqueue;
#endif

#ifdef NXE2000_USED_TIMER_THREAD
	struct timer_list			rdstate_timer;
	u32							rdstate_repeat_time;
#endif

	int			first_pwon;

	int			gpio_eint;
	int		    gpio_otg_usbid;
	int			gpio_otg_vbus;
	int			gpio_pmic_vbus;
	int			gpio_pmic_lowbat;

	int			bat_low_uV;
	int			bat_max_uV;
	int			bat_min_uV;

	bool		event_ubc_recheck;
	u8			power_src;
	int			ubc_state;

	bool		wall_online;
	bool		usb_online;
	char		wall_name[20];
	char		usb_name[20];
	char		battery_name[20];
	bool		have_battery;
	bool		otg_dev_mode;

	u8			chg_prog_bar_ratio;

	int			adp_ilim_current;

	int			usb_ilim_range_3A;
	int			usb_ilim_current;

	int			adp_chg_current;
	int			usb_chg_current;

	bool		power_prio_vusb;

#if 0
	uint8_t		slp_prio_buck[NXE2000_NUM_BUCK];	// 0 ~ 14, off => 15
	uint8_t		slp_prio_ldo[NXE2000_NUM_LDO];
	uint8_t		slp_prio_pso[NXE2000_NUM_PSO];		// power-supply shutoff 
#endif

};

static int nxe2000_bat_prop_is_writeable(struct power_supply *psy,
						enum power_supply_property prop)
{
	switch (prop) {
//	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
//	case POWER_SUPPLY_PROP_USB_INPRIORITY:
	case POWER_SUPPLY_PROP_AUTO_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property nxe2000_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,			/* "FULL" or "NOT FULL" only. */
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
//	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
//	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_PRESENT,			/* the presence of battery */
//	POWER_SUPPLY_PROP_ONLINE,			/* charger is active or not */
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
#if (CFG_NXE2000_DEVEL_MODE == 0)
	POWER_SUPPLY_PROP_REMOTE_TYPE,
#endif
	POWER_SUPPLY_PROP_AUTO_CURRENT_LIMIT,
};

struct current_map_desc {
	int min;
	int max;
	int step;
};

static volatile int nxe2000_power_suspend_status;
static volatile int nxe2000_power_resume_status;
static volatile int nxe2000_power_resume_check;
static volatile int nxe2000_power_lowbat;
static volatile int nxe2000_power_lowbat_resume_count;

/* current map in mA */
static const struct current_map_desc adp_ilim_current_map_desc = {
	.min = 100,		.max = 2500,		.step = 100,
};

static const struct current_map_desc usb_ilim_current_map_desc = {
	.min = 100,		.max = 1500,		.step = 100,
};

static const struct current_map_desc chg_current_map_desc = {
	.min = 100,		.max = 1800,		.step = 100,
};

static const struct current_map_desc *chg_current_map[] = {
	[NXE2000_PWR_SRC_BAT] = &chg_current_map_desc,			/* Charge current */
	[NXE2000_PWR_SRC_ADP] = &adp_ilim_current_map_desc,		/* ADP limit current */
	[NXE2000_PWR_SRC_USB] = &usb_ilim_current_map_desc,		/* USB limit current */
};

    
#if (CFG_NXE2000_BAT_CHG_SUPPORT == 1)
static int Calc_Linear_Interpolation(int x0, int y0, int x1, int y1, int y)
{
	int	alpha;
	int x;

	alpha = (y - y0)*100 / (y1 - y0);

	x = ((100 - alpha) * x0 + alpha * x1) / 100;

	return x;
}

#if 0
static int get_OCV_init_Data(struct ricoh61x_battery_info *info, int index)
{
	int ret = 0;
	ret =  (battery_init_para[info->num][index*2]<<8) | (battery_init_para[info->num][index*2+1]);
	return ret;
}

static int get_OCV_voltage(struct ricoh61x_battery_info *info, int index)
{
	int ret = 0;
	ret =  get_OCV_init_Data(info, index);
	/* conversion unit 1 Unit is 1.22mv (5000/4095 mv) */
	ret = ret * 50000 / 4095;
	/* return unit should be 1uV */
	ret = ret * 100;
	return ret;
}

static void nxe2000_scaling_OCV_table(struct ricoh61x_battery_info *info, int cutoff_vol, int full_vol, int *start_per, int *end_per)
{
	int		i, j;
	int		temp;
	int		percent_step;
	int		OCV_percent_new[11];

	/* Check Start % */
	if (info->soca->ocv_table_def[0] > cutoff_vol * 1000)
	{
		*start_per = 0;
	}
	else
	{
		for (i = 1; i < 11; i++)
		{
			if (info->soca->ocv_table_def[i] >= cutoff_vol * 1000) {
				/* unit is 0.001% */
				*start_per = Calc_Linear_Interpolation(
					(i-1)*1000, info->soca->ocv_table_def[i-1], i*1000,
					info->soca->ocv_table_def[i], (cutoff_vol * 1000));
				break;
			}
		}
	}

	/* Check End % */
	for (i = 1; i < 11; i++)
	{
		if (info->soca->ocv_table_def[i] >= full_vol * 1000)
		{
			/* unit is 0.001% */
			*end_per = Calc_Linear_Interpolation(
				(i-1)*1000, info->soca->ocv_table_def[i-1], i*1000,
				 info->soca->ocv_table_def[i], (full_vol * 1000));
			break;
		}
	}

	/* calc new ocv percent */
	percent_step = ( *end_per - *start_per) / 10;

	for (i = 0; i < 11; i++)
	{
		OCV_percent_new[i]
			 = *start_per + percent_step*(i - 0);
	}

	/* calc new ocv voltage */
	for (i = 0; i < 11; i++)
	{
		for (j = 1; j < 11; j++)
		{
			if (1000*j >= OCV_percent_new[i])
			{
				temp = Calc_Linear_Interpolation(
					info->soca->ocv_table_def[j-1], (j-1)*1000,
					info->soca->ocv_table_def[j] , j*1000,
					 OCV_percent_new[i]);

				temp = ( (temp/1000) * 4095 ) / 5000;

				battery_init_para[info->num][i*2 + 1] = temp;
				battery_init_para[info->num][i*2] = temp >> 8;

				break;
			}
		}
	}
	for (i = 0; i <= 10; i = i+1)
	{
		temp = (battery_init_para[info->num][i*2]<<8)
			 | (battery_init_para[info->num][i*2+1]);
		/* conversion unit 1 Unit is 1.22mv (5000/4095 mv) */
		temp = ((temp * 50000 * 10 / 4095) + 5) / 10;
	}
}

static int nxe2000_set_OCV_table(struct nxe2000_power *power)
{
	int		ret = 0;
	int		i;
	int		full_ocv;
	int		available_cap;
	int		available_cap_ori;
	int		temp;
	int		temp1;
	int		start_per = 0;
	int		end_per = 0;
	int		Rbat;
	int		Ibat_min;
	uint8_t val;
	uint8_t val2;
	uint8_t val_temp;

	//get ocv table 
	for (i = 0; i <= 10; i = i+1) {
		info->soca->ocv_table_def[i] = get_OCV_voltage(info, i);
	}

	temp    = (battery_init_para[info->num][24]<<8) | (battery_init_para[info->num][25]);
	Rbat    = temp * 1000 / 512 * 5000 / 4095;
	info->soca->Rsys = Rbat + 55;

	if ((info->fg_target_ibat == 0) || (info->fg_target_vsys == 0)) 	/* normal version */
	{
		temp    = (battery_init_para[info->num][22]<<8) | (battery_init_para[info->num][23]);

		info->soca->target_ibat = temp*2/10; /* calc 0.2C*/
		temp1   = (battery_init_para[info->num][0]<<8) | (battery_init_para[info->num][1]);

		info->soca->target_vsys = temp1 + ( info->soca->target_ibat * info->soca->Rsys ) / 1000;
	}
	else
	{
		info->soca->target_ibat = info->fg_target_ibat;

		/* calc min vsys value */
		temp1   = (battery_init_para[info->num][0]<<8) | (battery_init_para[info->num][1]);
		temp    = temp1 + ( info->soca->target_ibat * info->soca->Rsys ) / 1000;
		if( temp < info->fg_target_vsys)
		{
			info->soca->target_vsys = info->fg_target_vsys;
		}
		else
		{
			info->soca->target_vsys = temp;
		}
	}

	if ((info->soca->target_ibat != 0) && (info->soca->target_vsys != 0)) 	/*Slice cutoff voltage version. */
	{
		Ibat_min    = -1 * info->soca->target_ibat;
		info->soca->cutoff_ocv = info->soca->target_vsys - Ibat_min * info->soca->Rsys / 1000;
		
		full_ocv    = (battery_init_para[info->num][20]<<8) | (battery_init_para[info->num][21]);
		full_ocv    = full_ocv * 5000 / 4095;

		nxe2000_scaling_OCV_table(info, info->soca->cutoff_ocv, full_ocv, &start_per, &end_per);

		/* calc available capacity */
		/* get avilable capacity */
		/* battery_init_para23-24 is designe capacity */
		available_cap   = (battery_init_para[info->num][22]<<8)
					    | (battery_init_para[info->num][23]);

		available_cap = available_cap
			 * ((10000 - start_per) / 100) / 100 ;

		battery_init_para[info->num][23]    = available_cap;
		battery_init_para[info->num][22]    = available_cap >> 8;
	}

	ret = nxe2000_clr_bits(info->dev->parent, NXE2000_REG_FG_CTRL, 0x01);
	if (ret < 0) {
		dev_err(info->dev, "error in FG_En off\n");
		goto err;
	}

	/////////////////////////////////
	ret = nxe2000_read_bank1(info->dev->parent, 0xDC, &val);
	if (ret < 0) {
		dev_err(info->dev, "batterry initialize error\n");
		goto err;
	}

	val_temp = val;
	val	&= 0x0F;    //clear bit 4-7
	val	|= 0x10;    //set bit 4

	ret = nxe2000_write_bank1(info->dev->parent, 0xDC, val);
	if (ret < 0) {
		dev_err(info->dev, "batterry initialize error\n");
		goto err;
	}

	ret = nxe2000_read_bank1(info->dev->parent, 0xDC, &val2);
	if (ret < 0) {
		dev_err(info->dev, "batterry initialize error\n");
		goto err;
	}

	ret = nxe2000_write_bank1(info->dev->parent, 0xDC, val_temp);
	if (ret < 0) {
		dev_err(info->dev, "batterry initialize error\n");
		goto err;
	}

	if (val != val2)
	{
		ret = nxe2000_bulk_writes_bank1(info->dev->parent,
				NXE2000_REG_BAT_INIT_TOP, 30, battery_init_para[info->num]);
		if (ret < 0) {
			dev_err(info->dev, "batterry initialize error\n");
			goto err;
		}
	}
	else
	{
		ret = nxe2000_read_bank1(info->dev->parent, 0xD2, &val);
		if (ret < 0) {
			dev_err(info->dev, "batterry initialize error\n");
			goto err;
		}

		ret = nxe2000_read_bank1(info->dev->parent, 0xD3, &val2);
		if (ret < 0) {
			dev_err(info->dev, "batterry initialize error\n");
			goto err;
		}

		available_cap_ori   = val2 + (val << 8);
		available_cap       = battery_init_para[info->num][23]
						    + (battery_init_para[info->num][22] << 8);

		if (available_cap_ori == available_cap)
		{
			ret = nxe2000_bulk_writes_bank1(info->dev->parent,
				NXE2000_REG_BAT_INIT_TOP, 22, battery_init_para[info->num]);
			if (ret < 0) {
				dev_err(info->dev, "batterry initialize error\n");
				return ret;
			}

			for (i = 0; i < 6; i++)
			{
				ret = nxe2000_write_bank1(info->dev->parent, 0xD4+i, battery_init_para[info->num][24+i]);
				if (ret < 0) {
					dev_err(info->dev, "batterry initialize error\n");
					return ret;
				}
			}
		}
		else
		{
			ret = nxe2000_bulk_writes_bank1(info->dev->parent,
					NXE2000_REG_BAT_INIT_TOP, 30, battery_init_para[info->num]);
			if (ret < 0) {
				dev_err(info->dev, "batterry initialize error\n");
				goto err;
			}
		}
	}

	////////////////////////////////

	return 0;
err:
	return ret;
}
#else

static int nxe2000_set_OCV_table(struct nxe2000_power *power)
{
	int		ret = 0;
	int		ocv_table[11];
	int		i, j;
	int		available_cap;
	int		temp;
	int		start_par;
	int		percent_step;
	int		OCV_percent_new[11];

	/*Slice cutoff voltage version. */
	if (NXE2000_CUTOFF_VOL > 0)
	{
		/* get ocv table. this table is calculated by Apprication */
		for (i = 0; i < 11; i++) {
			temp = (battery_init_para[i*2]<<8)
				 | (battery_init_para[i*2+1]);
			/* conversion unit 1 Unit is 1.22mv (5000/4095 mv) */
			temp = ((temp * 50000 * 10 / 4095) + 5) / 10;
			ocv_table[i] = temp;
		}

		/* Check Start % */
		for (i = 1; i < 11; i++) {
			if (ocv_table[i] >= NXE2000_CUTOFF_VOL * 10) {
				/* unit is 0.001% */
				start_par = Calc_Linear_Interpolation(
					(i-1)*1000, ocv_table[i-1], i*1000,
					 ocv_table[i], (NXE2000_CUTOFF_VOL * 10));
				break;
			}
		}

		/* calc new ocv percent */
		percent_step = (10000 - start_par) / 10;

		for (i = 0; i < 11; i++) {
			OCV_percent_new[i]
				 = start_par + percent_step*(i - 0);
		}

		/* calc new ocv voltage */
		for (i = 0; i < 11; i++) {
			for (j = 1; j < 11; j++) {
				if (1000*j >= OCV_percent_new[i]) {
					temp = Calc_Linear_Interpolation(
						ocv_table[j-1], (j-1)*1000,
						 ocv_table[j] , j*1000,
						 OCV_percent_new[i]);

					temp = temp * 4095 / 50000;

					battery_init_para[i*2 + 1] = temp;
					battery_init_para[i*2] = temp >> 8;

					j = 11;
				}
			}
		}

		/* calc available capacity */
		/* get avilable capacity */
		/* battery_init_para23-24 is designe capacity */
		available_cap = (battery_init_para[22]<<8)
					 | (battery_init_para[23]);

		available_cap = available_cap
			 * ((10000 - start_par) / 100) / 100 ;

		battery_init_para[23] =  available_cap;
		battery_init_para[22] =  available_cap >> 8;
	}

	ret = nxe2000_bulk_writes_bank1(power->iodev->i2c,
			 NXE2000_REG_BAT_INIT_TOP, 32, battery_init_para);
	if (ret < 0) {
		dev_err(power->dev, "batterry initialize error\n");
		return ret;
	}

	return 1;
}
#endif

/* Initial setting of battery */
static int nxe2000_init_battery(struct nxe2000_power *power)
{
	int ret = 0;
	uint8_t val;
	uint8_t val2;

	/* Need to implement initial setting of batery and error */
	/* -------------------------- */
#if 1   //def ENABLE_FUEL_GAUGE_FUNCTION

	/* set relaxation state */
	if (NXE2000_REL1_SEL_VALUE > 240)
		val = 0x0F;
	else
		val = NXE2000_REL1_SEL_VALUE / 16 ;

	/* set relaxation state */
	if (NXE2000_REL2_SEL_VALUE > 120)
		val2 = 0x0F;
	else
		val2 = NXE2000_REL2_SEL_VALUE / 8 ;

	val =  val + (val2 << 4);

	ret = nxe2000_write_bank1(power->iodev->i2c, NXE2000_REG_BAT_REL_SEL, val);
	if (ret < 0) {
		dev_err(power->dev, "Error in writing NXE2000_REG_BAT_REL_SEL register\n");
		return ret;
	}

	ret = nxe2000_write_bank1(power->iodev->i2c, NXE2000_REG_BAT_TA_SEL, 0x00);
	if (ret < 0) {
		dev_err(power->dev, "Error in writing NXE2000_REG_BAT_TA_SEL register\n");
		return ret;
	}

	ret = nxe2000_read(power->iodev->i2c, NXE2000_REG_FG_CTRL, &val);
	if (ret < 0) {
		dev_err(power->dev, "Error in reading the control register\n");
		return ret;
	}
	val = (val >> NXE2000_POS_FG_CTRL_FG_ACC) & 1;
	power->first_pwon = (val == 0) ? 1 : 0;

#if 1
	ret = nxe2000_write(power->iodev->i2c, NXE2000_REG_FG_CTRL, 0x00);
	if (ret < 0) {
		dev_err(power->dev, "Error in writing the control register\n");
		return ret;
	}
#endif

	ret = nxe2000_set_OCV_table(power);
	if (ret < 0) {
		dev_err(power->dev, "Error in writing the OCV Tabler\n");
		return ret;
	}

	ret = nxe2000_read(power->iodev->i2c, NXE2000_REG_CHGSTATE, &val);
	if (ret < 0) {
		dev_err(power->dev, "Error in reading the status register\n");
		return ret;
	}
	power->iodev->status_regs[NXE2000_IRQGRP_FG_INT] = val;

	if (power->first_pwon)
		val = 0x91;
	else
		val = 0x51;

	ret = nxe2000_write(power->iodev->i2c, NXE2000_REG_FG_CTRL, 0x11);
	if (ret < 0) {
		dev_err(power->dev, "Error in writing the control register\n");
		return ret;
	}

#if 1
	do {
		nxe2000_read(power->iodev->i2c, NXE2000_REG_FG_CTRL, &val);
	} while(val & 0xC0);
#endif
#endif

#if 0
	ret = nxe2000_write(power->iodev->i2c, NXE2000_REG_VINDAC, 0x01);
	if (ret < 0) {
		dev_err(power->dev, "Error in writing NXE2000_REG_VINDAC register\n");
		return ret;
	}
#endif

#if 0
	if (info->alarm_vol_mv < 2700 || info->alarm_vol_mv > 3400) {
		dev_err(power->dev, "alarm_vol_mv is out of range!\n");
		return -1;
	}
#endif

	return ret;
}
#endif  // #if (CFG_NXE2000_BAT_CHG_SUPPORT == 0)

static u8 nxe2000_get_current_proper_val(
		const struct current_map_desc *desc,
		int min_curr, int max_curr)
{
	int i = 0;

	if (desc == NULL)
		return 0;

	if (max_curr < desc->min || min_curr > desc->max)
		return 0;

	while (desc->min + desc->step * i < min_curr &&
			desc->min + desc->step * i < desc->max)
		i++;

	if (desc->min + desc->step * i > max_curr)
		return 0;

	return i;
}

static int nxe2000_set_chg_current(
		struct i2c_client *i2c,
		enum nxe2000_pwr_src supply,
		u32 uA,
		u8  misc)
{
	const struct current_map_desc *desc;
	int min_mA, max_mA;
	u8	value;

	desc	= chg_current_map[supply];

	value	= (uA / 100000);
	min_mA	= value * 100;
	max_mA	= (value + 1) * 100;

	value = nxe2000_get_current_proper_val(desc, min_mA, max_mA) | misc;

	switch (supply)
	{
	case NXE2000_PWR_SRC_BAT:
		nxe2000_update(i2c, NXE2000_REG_CHGISET, value, NXE2000_VAL_CHGISET_ICHG_MASK);
		break;
	case NXE2000_PWR_SRC_ADP:
		nxe2000_write(i2c, NXE2000_REG_REGISET1, value);
		break;
	case NXE2000_PWR_SRC_USB:
		nxe2000_write(i2c, NXE2000_REG_REGISET2, value);
		break;
	}

	return 0;
}

static int nxe2000_check_online(
		u8 chg_state,
		enum nxe2000_pwr_src supply,
		union power_supply_propval *val)
{
	u8	value;

	val->intval = 0;

	switch (supply)
	{
	case NXE2000_PWR_SRC_BAT:
		value = chg_state & NXE2000_POS_CHGSTATE_RDSTATE_MASK;
		if (value != NXE2000_VAL_CHGSTATE_NO_BATT && value != NXE2000_VAL_CHGSTATE_NO_BATT2)
			val->intval = 1;
		break;
	case NXE2000_PWR_SRC_ADP:
	case NXE2000_PWR_SRC_USB:
		value = ((chg_state >> NXE2000_POS_CHGSTATE_USEADP) & supply);
		if (value)
			val->intval = 1;
		break;
	}

	return 0;
}

static void nxe2000_config_battery(struct nxe2000_dev *iodev)
{
#if 0
	struct nxe2000_pdata *nxe2000_pdata = iodev->dev->platform_data;

	if (!nxe2000_pdata) {
		dev_warn(iodev->dev,
			 "No battery charger configuration\n");
		return;
	}
#endif


#ifndef	NXE2000_USED_TIMER_THREAD
#ifdef	NXE2000_RDSTATE_ONLY
	nxe2000_write(iodev->i2c, NXE2000_REG_CHGCTL_IRFMASK,	0x3F);
#else
	nxe2000_write(iodev->i2c, NXE2000_REG_CHGCTL_IRFMASK,	0x00);
	nxe2000_write(iodev->i2c, NXE2000_REG_CHGSTAT_IRFMASK1,	0xF0);
	nxe2000_write(iodev->i2c, NXE2000_REG_CHGSTAT_IRFMASK2,	0xF0);
	nxe2000_write(iodev->i2c, NXE2000_REG_CHGERR_IRFMASK,	0x00);
#endif
#endif
}

static int nxe2000_bat_check_status(u8 chg_state, int *status)
{
	switch (chg_state & NXE2000_POS_CHGSTATE_RDSTATE_MASK) {
	case NXE2000_VAL_CHGSTATE_DIE_SHUDN:
	case NXE2000_VAL_CHGSTATE_VCHG_OVER:
	case NXE2000_VAL_CHGSTATE_SUSPEND:
	case NXE2000_VAL_CHGSTATE_CHG_OFF:
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case NXE2000_VAL_CHGSTATE_CHG_CMP:
		*status = POWER_SUPPLY_STATUS_FULL;
		break;
	case NXE2000_VAL_CHGSTATE_NO_BATT2:
	case NXE2000_VAL_CHGSTATE_NO_BATT:
		if (chg_state & NXE2000_POS_CHGSTATE_PWRSRC_MASK)
			*status = POWER_SUPPLY_STATUS_UNKNOWN;
		else
			*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case NXE2000_VAL_CHGSTATE_DIE_ERR:
	case NXE2000_VAL_CHGSTATE_BATT_ERR:
	case NXE2000_VAL_CHGSTATE_BATT_TEMPERR:
	case NXE2000_VAL_CHGSTATE_BATT_OVV:
	case NXE2000_VAL_CHGSTATE_CHG_RDY_VUSB:
	case NXE2000_VAL_CHGSTATE_CHG_RDY_VADP:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case NXE2000_VAL_CHGSTATE_TRICKLE_CHG:
	case NXE2000_VAL_CHGSTATE_RAPID_CHG:
		*status = POWER_SUPPLY_STATUS_CHARGING;
		break;

	default:
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

static int nxe2000_bat_charge_type(u8 chg_state, int *type)
{
	switch (chg_state & NXE2000_POS_CHGSTATE_RDSTATE_MASK) {
	case NXE2000_VAL_CHGSTATE_TRICKLE_CHG:
		*type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case NXE2000_VAL_CHGSTATE_RAPID_CHG:
		*type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	default:
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	}

	return 0;
}

#if 0
static int nxe2000_battery_check_health(struct i2c_client *i2c, int *health)
{
	int ret;

	ret = wm831x_reg_read(wm831x, WM831X_CHARGER_STATUS);
	if (ret < 0)
		return ret;

	if (ret & WM831X_BATT_HOT_STS) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		return 0;
	}

	if (ret & WM831X_BATT_COLD_STS) {
		*health = POWER_SUPPLY_HEALTH_COLD;
		return 0;
	}

	if (ret & WM831X_BATT_OV_STS) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}

	switch (ret & WM831X_CHG_STATE_MASK) {
	case WM831X_CHG_STATE_TRICKLE_OT:
	case WM831X_CHG_STATE_FAST_OT:
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case WM831X_CHG_STATE_DEFECTIVE:
		*health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	default:
		*health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	}

	return 0;
}
#endif

static int nxe2000_apsd_detect(struct nxe2000_power	*nxe2000_power,
					union power_supply_propval *val)
{
	struct i2c_client       *i2c = nxe2000_power->iodev->i2c;
//    struct nxe2000_pdata    *pdata = &nxe2000_power->platdata;

	int	chg_current = 0;

	int ret = 0;
	int retry = 30;
    u8  value;
    u8  chg_state = nxe2000_power->iodev->status_regs[NXE2000_IRQGRP_FG_INT];

	val->intval = 0;

//	mutex_lock(&nxe2000_power->lock);

#if 1
#if 0
	value = (1 << NXE2000_POS_GCHGDET_DET_RESTART);
	ret  = nxe2000_write(i2c, NXE2000_REG_EXTIF_GCHGDET, value);
    udelay(10);
#endif

recheck_detect:
	ret = nxe2000_read(i2c, NXE2000_REG_EXTIF_GCHGDET, &value);
    if ( (ret || (value & 0x01)) && retry )
    {
        udelay(10);
        retry--;
        goto recheck_detect;
    }

    if ( !retry )
    {
        val->intval = POWER_SUPPLY_TYPE_BATTERY;
        ret = -1;
        goto apsd_fail;
    }

	retry = 30;
#else

    value = (1 << NXE2000_POS_GCHGDET_DET_RESTART);
    ret = nxe2000_write(i2c, NXE2000_REG_EXTIF_GCHGDET, value);
	mdelay(30);
#endif

recheck_ubc:
	mdelay(10);
//	msleep(10);

    ret = nxe2000_read(i2c, NXE2000_REG_EXTIF_GCHGDET, &value);
	msleep(1);
    if (ret != 0)
    {
        if (retry)
        {
            retry--;
            goto recheck_ubc;
        }
        else
        {
            val->intval = POWER_SUPPLY_TYPE_BATTERY;
            ret = -1;
            goto apsd_fail;
        }
    }

#if 1
    if ( (value & 0xC4) && retry )
    {
//		msleep(10);
        retry--;
        goto recheck_ubc;
    }
#endif

#if 1
    if ( ((value >> NXE2000_POS_GCHGDET_GC_DET) & NXE2000_GCHGDET_GC_DET_MASK) != NXE2000_GCHGDET_GC_DET_COMPLETE )
    {
        if ( (value & 0xC0) && retry )
		{
            value = (1 << NXE2000_POS_GCHGDET_DET_RESTART);
            ret = nxe2000_write(i2c, NXE2000_REG_EXTIF_GCHGDET, value);
//            msleep(10);

	        retry--;
	        goto recheck_ubc;
        }
        else
        {
            val->intval = POWER_SUPPLY_TYPE_BATTERY;
            ret = -1;
            goto apsd_fail;
        }
    }
	else
	{
        if ( (value & 0xC0) && retry )
		{
            value = (1 << NXE2000_POS_GCHGDET_DET_RESTART);
            ret = nxe2000_write(i2c, NXE2000_REG_EXTIF_GCHGDET, value);
//            msleep(10);

	        retry--;
	        goto recheck_ubc;
        }
	}
#endif


	nxe2000_power->ubc_state = (value >> NXE2000_POS_GCHGDET_VBUS_TYPE) & NXE2000_GCHGDET_VBUS_TYPE_MASK;
#if 0
	switch (nxe2000_power->ubc_state)
	{
	case NXE2000_GCHGDET_VBUS_TYPE_CDP:
		val->intval = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case NXE2000_GCHGDET_VBUS_TYPE_DCP:
		val->intval = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case NXE2000_GCHGDET_VBUS_TYPE_SDP:
		val->intval = POWER_SUPPLY_TYPE_USB;
		break;
	default:
		val->intval = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	}
#endif

	switch (nxe2000_power->ubc_state)
	{
	case NXE2000_GCHGDET_VBUS_TYPE_CDP:
	case NXE2000_GCHGDET_VBUS_TYPE_DCP:

        chg_current = nxe2000_power->adp_chg_current;
        nxe2000_power->power_prio_vusb  = false;
		break;
    case NXE2000_GCHGDET_VBUS_TYPE_SDP:
    case NXE2000_GCHGDET_VBUS_TYPE_OTHER:
    default:
        chg_current = nxe2000_power->usb_chg_current;
        nxe2000_power->power_prio_vusb  = true;
        break;
	}

#if (CFG_NXE2000_ADP_ONLY_MODE == 1)
    chg_current = nxe2000_power->adp_chg_current;
    nxe2000_power->power_prio_vusb  = false;
#endif

    nxe2000_set_chg_current(i2c, NXE2000_PWR_SRC_BAT,
            chg_current, 0);
    nxe2000_set_chg_current(i2c, NXE2000_PWR_SRC_ADP,
            nxe2000_power->adp_ilim_current, 0);
    nxe2000_set_chg_current(i2c, NXE2000_PWR_SRC_USB,
            nxe2000_power->usb_ilim_current, 0);

apsd_fail:
//	mutex_unlock(&nxe2000_power->lock);

    if ( !retry )
    {
        value = (1 << NXE2000_POS_GCHGDET_DET_RESTART);
        ret  = nxe2000_write(i2c, NXE2000_REG_EXTIF_GCHGDET, value);
    }

	return ret;
}


/*********************************************************************
 *		WALL Power
 *********************************************************************/
static int nxe2000_wall_get_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct nxe2000_power	*nxe2000_power = dev_get_drvdata(psy->dev->parent);
	struct i2c_client		*i2c = nxe2000_power->iodev->i2c;
	int	ret = 0;
	u8	chg_state;

	chg_state = nxe2000_power->iodev->status_regs[NXE2000_IRQGRP_FG_INT];

	mutex_lock(&nxe2000_power->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = nxe2000_check_online(chg_state, NXE2000_PWR_SRC_ADP, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = nxe2000_get_bat_voltage(i2c, val);
		break;
    case POWER_SUPPLY_PROP_CURRENT_MAX:
        val->intval = nxe2000_power->adp_ilim_current;
        break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&nxe2000_power->lock);

	nxe2000_power_resume_check  = 0;

	return ret;
}

static enum power_supply_property nxe2000_wall_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

/*********************************************************************
 *		USB Power
 *********************************************************************/
static int nxe2000_usb_get_prop(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct nxe2000_power	*nxe2000_power = dev_get_drvdata(psy->dev->parent);
	struct i2c_client		*i2c = nxe2000_power->iodev->i2c;
	int	ret = 0;
	u8	chg_state;

	chg_state = nxe2000_power->iodev->status_regs[NXE2000_IRQGRP_FG_INT];

	mutex_lock(&nxe2000_power->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = nxe2000_check_online(chg_state, NXE2000_PWR_SRC_USB, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = nxe2000_get_bat_voltage(i2c, val);
		break;
    case POWER_SUPPLY_PROP_CURRENT_MAX:
        val->intval = nxe2000_power->usb_ilim_current;
        break;
#if 0
	case POWER_SUPPLY_PROP_REMOTE_TYPE:
        ret = nxe2000_apsd_detect(nxe2000_power, val);
        break;
#endif
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&nxe2000_power->lock);
	return ret;
}

static enum power_supply_property nxe2000_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
//    POWER_SUPPLY_PROP_REMOTE_TYPE,
};

/*********************************************************************
 *		Battery properties
 *********************************************************************/

static int nxe2000_get_bat_capacity(struct i2c_client *i2c,
				     union power_supply_propval *val)
{
	u8  value[2];
	int	ret;

	val->intval = 0;

	ret = nxe2000_bulk_reads(i2c, NXE2000_REG_RE_CAP_H, 2, value);
	if (ret)
		goto exit_read_capacity;

	val->intval = (value[0] << 8) | value[1];

exit_read_capacity:
	return ret;
}

static int nxe2000_get_bat_voltage(struct i2c_client *i2c,
				     union power_supply_propval *val)
{
	u8  value[2];
	int	data, ret;

	val->intval = 0;

	ret = nxe2000_bulk_reads(i2c, NXE2000_REG_VOLTAGE_1, 2, value);
	if (ret)
		goto exit_read_voltage;

	data = (value[0] << 8) | value[1];

	/* conversion unit 1 Unit is 1.22mV (5000/4095 mV) */
	data = data * 50000 / 4095;

	/* return unit should be 1uV */
	val->intval = data * 100;

exit_read_voltage:
	return ret;
}

static int nxe2000_get_bat_time2empty(struct i2c_client *i2c,
				     union power_supply_propval *val)
{
	u8  value[2];
	int	ret;

	val->intval = 0;

	ret = nxe2000_bulk_reads(i2c, NXE2000_REG_TT_EMPTY_H, 2, value);
	if (ret)
		goto exit_get_time2empty;

	val->intval = ((value[0] << 8) | value[1]) * 60;	/* unit is minute */

exit_get_time2empty:
	return ret;
}

static int nxe2000_get_bat_time2full(struct i2c_client *i2c,
				     union power_supply_propval *val)
{
	u8  value[2];
	int	ret;

	val->intval = 0;

	ret = nxe2000_bulk_reads(i2c, NXE2000_REG_TT_FULL_H, 2, value);
	if (ret)
		goto exit_get_time2full;

	val->intval = ((value[0] << 8) | value[1]) * 60;	/* unit is minute */

exit_get_time2full:
	return ret;
}

static int nxe2000_get_bat_temp(struct i2c_client *i2c,
				     union power_supply_propval *val)
{
	u8  value[2];
	int	data;
	int ret;

	val->intval = 0;

	ret = nxe2000_bulk_reads(i2c, NXE2000_REG_TEMP_1, 2, value);
	if (ret)
		goto exit_get_temp;

	data = ((value[0] << 8) | value[1]) & 0x07FF;	/* unit : 0.0625 degree */
    if (value[0] & 0x08)    /* signed bit */
    {
    	val->intval = (-1 * data * 625) / 10000;
    }
    else
    {
    	val->intval = (data * 625) / 10000;
    }

exit_get_temp:
	return ret;
}

static int nxe2000_bat_get_prop(
		struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct nxe2000_power	*nxe2000_power = dev_get_drvdata(psy->dev->parent);
//	struct nxe2000_pdata	*pdata = &nxe2000_power->platdata;
	struct i2c_client		*i2c = nxe2000_power->iodev->i2c;
    bool otg_dev_mode;
	int	ret = 0;
	u8	reg, chg_state;

	DBGOUT("%s\n", __func__);

#if 0
	if (nxe2000_power_suspend_status)
    {   
		return -EBUSY;
    }
#endif

	mutex_lock(&nxe2000_power->lock);

	chg_state = nxe2000_power->iodev->status_regs[NXE2000_IRQGRP_FG_INT];

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
//        nxe2000_apsd_detect(nxe2000_power, val);
#ifdef	NXE2000_USED_TIMER_THREAD
		ret = nxe2000_read(i2c, NXE2000_REG_CHGSTATE,
				&chg_state);
		if (ret) {
			val->intval = 0;
		}
		nxe2000_power->iodev->status_regs[NXE2000_IRQGRP_FG_INT] = chg_state;
#endif

#if defined(CONFIG_USB_DWCOTG)

        if (nxe2000_power->gpio_otg_usbid > -1)
            otg_dev_mode = gpio_get_value(nxe2000_power->gpio_otg_usbid);
        else
            otg_dev_mode = 1;

        if( !otg_dev_mode && nxe2000_power->otg_dev_mode )
        {
            nxe2000_power->power_src            = NXE2000_PWR_SRC_BAT;
            nxe2000_power->event_ubc_recheck    = true;
        }
        else
#endif
        {
            reg = chg_state & NXE2000_POS_CHGSTATE_PWRSRC_MASK;
    		if (nxe2000_power->power_src != reg)
    		{
        		nxe2000_power->power_src            = reg;
                nxe2000_power->event_ubc_recheck    = true;
    		}
        }

        nxe2000_power->otg_dev_mode = otg_dev_mode;

#if 0
        if (nxe2000_power->event_ubc_recheck == true)
        {
            u8 mask;

            reg     = (NXE2000_CHG_PRIO << NXE2000_POS_CHGCTL1_CHGP)
                    | (0x1 << NXE2000_POS_CHGCTL1_SUSPEND);
            nxe2000_update(i2c, NXE2000_REG_CHGCTL1, reg, NXE2000_CHG_MASK);
        }
#endif

		ret = nxe2000_bat_check_status(chg_state, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = nxe2000_read(i2c, NXE2000_REG_CHGERR_MONI, &reg);
		if (ret)
			return ret;
		if (reg & (NXE2000_POS_CHGERR_MONI_DIEERRINT | NXE2000_POS_CHGERR_MONI_DIEOFFINT))
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:	/* POWER_SUPPLY_STATUS_UNKNOWN, CHARGING, DISCHARGING, NOT_CHARGING, FULL */
#if 1
		switch (chg_state & NXE2000_POS_CHGSTATE_RDSTATE_MASK) {
		case NXE2000_VAL_CHGSTATE_NO_BATT2:
		case NXE2000_VAL_CHGSTATE_NO_BATT:
			val->intval = 0;
			break;
		default:
			val->intval = 1;
		}
#else

		val->intval = 1;
#endif
		break;
	case POWER_SUPPLY_PROP_ONLINE:	/* 0: OFF Line, 1: ON Line  */
		ret = nxe2000_check_online(chg_state, NXE2000_PWR_SRC_BAT, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = nxe2000_get_bat_voltage(i2c, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = nxe2000_power->bat_max_uV;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = nxe2000_power->bat_min_uV;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = nxe2000_bat_charge_type(chg_state, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
        #if 0
		if (chg_state & (1 << NXE2000_POS_CHGSTATE_USEADP))
			val->intval = nxe2000_power->adp_ilim_current;
		else
			val->intval = nxe2000_power->usb_ilim_current;
        #else
		if (nxe2000_power->power_prio_vusb)
			val->intval = nxe2000_power->usb_ilim_current;
		else
			val->intval = nxe2000_power->adp_ilim_current;
        #endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = nxe2000_read(i2c, NXE2000_REG_CHGISET, &reg);
		if (ret)
			val->intval = 0;
		else
			val->intval = ((reg & NXE2000_VAL_CHGISET_ICHG_MASK) + 1) * 100000;	/* unit is mA */
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#if 0
		ret = nxe2000_get_bat_capacity(i2c, val);
		break;
#endif        
	case POWER_SUPPLY_PROP_CHARGE_NOW:
#if 1
		if (nxe2000_power_lowbat)
		{
			val->intval = (5 * nxe2000_power_lowbat);		/* unit is % */
			nxe2000_power_lowbat--;
			break;
		}
#endif

		if ((chg_state & NXE2000_POS_CHGSTATE_RDSTATE_MASK) == NXE2000_VAL_CHGSTATE_CHG_CMP)
		{
			val->intval = 100;			/* unit is % */
		}
		else
		{
/* Charging progressive bar */
#if (CFG_NXE2000_CHG_PROG_MODE == 1)
            if (chg_state & NXE2000_POS_CHGSTATE_PWRSRC_MASK)
            {
                if (nxe2000_power->chg_prog_bar_ratio > 8)
                {
                    nxe2000_power->chg_prog_bar_ratio = 0;
                }
                val->intval = (nxe2000_power->chg_prog_bar_ratio * 11) + 11;
                nxe2000_power->chg_prog_bar_ratio++;
            }
            else
#endif
            {
                u8 u8Temp, u8Temp1;

                nxe2000_read(i2c, NXE2000_REG_CHGISET, &u8Temp1);
                nxe2000_write(i2c, NXE2000_REG_CHGISET, 0);

                nxe2000_read(i2c, NXE2000_REG_CHGCTL1, &u8Temp);
                reg = (0x1 << NXE2000_POS_CHGCTL1_CHGP)
                    | (0x1 << NXE2000_POS_CHGCTL1_VUSBCHGEN)
                    //| (0x1 << NXE2000_POS_CHGCTL1_VADPCHGEN)
                    | (0x1 << NXE2000_POS_CHGCTL1_JEITAEN)
                    | (0x1 << NXE2000_POS_CHGCTL1_SUSPEND);
                nxe2000_write(i2c, NXE2000_REG_CHGCTL1, reg);

    			ret = nxe2000_read(i2c, NXE2000_REG_SOC, &reg);
    			if (ret)
    			{
    				val->intval = 0;
    			}
    			else
    			{
#if 1
					val->intval = reg;		/* unit is % */
#else
    				if (reg < 20)
    					val->intval = 5;		/* unit is % */
    				else if (reg < 25)
    					val->intval = 10;		/* unit is % */
    				else
    					val->intval = reg - 10;	/* unit is % */
#endif
    			}

                nxe2000_write(i2c, NXE2000_REG_CHGCTL1, u8Temp);
                nxe2000_write(i2c, NXE2000_REG_CHGISET, u8Temp1);
            }
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if ( !gpio_get_value(nxe2000_power->gpio_pmic_lowbat) )
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
        else if ( !gpio_get_value(nxe2000_power->gpio_eint) )
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = nxe2000_get_bat_temp(i2c, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = nxe2000_get_bat_time2empty(i2c, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = nxe2000_get_bat_time2full(i2c, val);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "NXE2000";
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Nexell";
		break;

#if 0   //(CFG_NXE2000_DEVEL_MODE == 1)
	case POWER_SUPPLY_PROP_REMOTE_TYPE:
    	nxe2000_apsd_detect(nxe2000_power, val);
        break;
#endif

#if (CFG_NXE2000_DEVEL_MODE == 0)
    case POWER_SUPPLY_PROP_REMOTE_TYPE:
//        chg_state = nxe2000_power->iodev->status_regs[NXE2000_IRQGRP_FG_INT];
        if (nxe2000_power->event_ubc_recheck == true)
        {
            u8 mask;

#if defined(CONFIG_USB_DWCOTG)
#if 1
            otg_clk_disable();
            otg_phy_off();
#endif
            msleep(10);
#endif  /* CONFIG_USB_DWCOTG */

            //if ((nxe2000_power->gpio_otg_vbus > -1) && (nxe2000_power->otg_dev_mode == gpio_get_value(nxe2000_power->gpio_otg_usbid)))
            if ((nxe2000_power->gpio_otg_vbus > -1) && (nxe2000_power->gpio_otg_usbid > -1))
            {
//                if (nxe2000_power->otg_dev_mode != gpio_get_value(nxe2000_power->gpio_otg_usbid))
                    nxe2000_power->otg_dev_mode = gpio_get_value(nxe2000_power->gpio_otg_usbid);

                if (nxe2000_power->otg_dev_mode)
                    gpio_set_value(nxe2000_power->gpio_otg_vbus, 0);
                else
                    gpio_set_value(nxe2000_power->gpio_otg_vbus, 1);
            }

#if defined(CONFIG_USB_DWCOTG)
            reg     = (0x1 << NXE2000_POS_CHGCTL1_CHGP)
                    | (0x1 << NXE2000_POS_CHGCTL1_SUSPEND);
            nxe2000_update(i2c, NXE2000_REG_CHGCTL1, reg, NXE2000_CHG_MASK);
#endif  /* CONFIG_USB_DWCOTG */
            mdelay(30);

            nxe2000_apsd_detect(nxe2000_power, val);

#if 0
//            if (nxe2000_power->ubc_state >= 0)
            {
                if (nxe2000_power->chg_change)
                {
                    nxe2000_set_chg_current(i2c, NXE2000_PWR_SRC_BAT,
                            nxe2000_power->chg_current, 0);
                    nxe2000_power->chg_change       = false;
                }
#if 1
                if (nxe2000_power->adp_ilim_change)
                {
                    nxe2000_set_chg_current(i2c, NXE2000_PWR_SRC_ADP,
                            nxe2000_power->adp_ilim_current, 0);
                    nxe2000_power->adp_ilim_change  = false;
                }
                if (nxe2000_power->usb_ilim_change)
                {
                    nxe2000_set_chg_current(i2c, NXE2000_PWR_SRC_USB,
                            nxe2000_power->usb_ilim_current,
                            (nxe2000_power->usb_ilim_range_3A<<NXE2000_POS_REGISET2_SDPOVRLIM) );
                    nxe2000_power->usb_ilim_change  = false;
                }
#endif
            }
#endif

            if ((nxe2000_power->ubc_state >= 0) && (chg_state & NXE2000_POS_CHGSTATE_PWRSRC_MASK))
            {
#if 0
#if defined(CONFIG_USB_DWCOTG)
                nxe2000_power->otg_dev_mode = gpio_get_value(nxe2000_power->gpio_otg_usbid);
                if (nxe2000_power->otg_dev_mode)
                {
                    reg     = (NXE2000_CHG_PRIO << NXE2000_POS_CHGCTL1_CHGP)
                            | (0x1 << NXE2000_CHG_PRIO);
                }
                else
#endif
                {
                    reg     = (NXE2000_CHG_PRIO << NXE2000_POS_CHGCTL1_CHGP)
                            | (0x1 << NXE2000_POS_CHGCTL1_SUSPEND)
                            | (0x1 << NXE2000_CHG_PRIO);
                }

                nxe2000_write(i2c, NXE2000_REG_CHGCTL1, reg);
#else

#if defined(CONFIG_USB_DWCOTG)
                nxe2000_power->otg_dev_mode = gpio_get_value(nxe2000_power->gpio_otg_usbid);
                if (nxe2000_power->otg_dev_mode)
                {
                    switch (nxe2000_power->ubc_state)
                    {
                    case NXE2000_GCHGDET_VBUS_TYPE_CDP:
                    case NXE2000_GCHGDET_VBUS_TYPE_DCP:
                        reg     = (0x1 << NXE2000_POS_CHGCTL1_VADPCHGEN)
                                | (0x1 << NXE2000_POS_CHGCTL1_JEITAEN);
                        break;
                    case NXE2000_GCHGDET_VBUS_TYPE_SDP:
                    default:
                        reg     = (0x1 << NXE2000_POS_CHGCTL1_CHGP)
                                | (0x1 << NXE2000_POS_CHGCTL1_JEITAEN)
                                | (0x1 << NXE2000_POS_CHGCTL1_VUSBCHGEN);
                        break;
                    }

#if (CFG_NXE2000_ADP_ONLY_MODE == 1)
                    reg = (0x1 << NXE2000_POS_CHGCTL1_VADPCHGEN);
#endif
                }
                else
#endif
                {
                    switch (nxe2000_power->ubc_state)
                    {
                    case NXE2000_GCHGDET_VBUS_TYPE_CDP:
                    case NXE2000_GCHGDET_VBUS_TYPE_DCP:
                        reg     = (0x1 << NXE2000_POS_CHGCTL1_SUSPEND);
                        break;
                    case NXE2000_GCHGDET_VBUS_TYPE_SDP:
                    default:
                        reg     = (0x1 << NXE2000_POS_CHGCTL1_CHGP)
                                | (0x1 << NXE2000_POS_CHGCTL1_SUSPEND);
                        break;
                    }
                }

                nxe2000_update(i2c, NXE2000_REG_CHGCTL1, reg, NXE2000_CHG_MASK);
#endif
            }

nxe2000_read(i2c, NXE2000_REG_CHGCTL1, &mask);

//OTG_PHY_ON_OFF:

            if (nxe2000_power->ubc_state >= 0)
            {
#if defined(CONFIG_USB_DWCOTG)
                switch (nxe2000_power->ubc_state)
                {
                case 0x0beeeef:
                    if (chg_state)
                        break;
                case NXE2000_GCHGDET_VBUS_TYPE_SDP:
                case NXE2000_GCHGDET_VBUS_TYPE_OTHER:
                    if (nxe2000_power->otg_dev_mode
                        && (nxe2000_power->power_src == NXE2000_PWR_SRC_BAT))
                    {
//                        nxp4330_otg_set_usb_state(false);
                        break;
                    }
                case NXE2000_GCHGDET_VBUS_TYPE_CDP:
#if 1
                    otg_clk_enable();
                    otg_phy_init();
#endif

                    break;
                }
#endif
            }

            nxe2000_power->event_ubc_recheck = false;
        }
        else
        {
            val->intval = POWER_SUPPLY_TYPE_BATTERY;
        }
        break;
#endif  // #if (CFG_NXE2000_DEVEL_MODE == 0)

	case POWER_SUPPLY_PROP_AUTO_CURRENT_LIMIT:
		val->intval = true;
		break;

	default:
#if 0
    	mutex_unlock(&nxe2000_power->lock);
		return -EINVAL;
#else
		ret = -EINVAL;
		break;
#endif
	}

    if (ret != 0)
        ret = -EINVAL;

	mutex_unlock(&nxe2000_power->lock);

//	return 0;
	return ret;
}

static int nxe2000_bat_set_prop(
		struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	int ret = -EINVAL;
#if 0
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, battery);
#endif

	switch (prop) {
#if 0
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		ret = smb347_charging_set(smb, val->intval);
		break;

	case POWER_SUPPLY_PROP_USB_INPRIORITY:
		smb347_set_writable(smb, true);

		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto priority_fail;
		ret |= CMD_A_SUSPEND_ENABLED;
		if (val->intval)
			ret &= ~CMD_A_SUSPEND_ENABLED;
		ret = smb347_write(smb, CMD_A, ret);
		if (ret < 0)
			goto priority_fail;

		ret = smb347_read(smb, CFG_VARIOUS_FUNCTION);
		if (ret < 0)
			goto priority_fail;
		ret &= ~(CFG_INPUT_SOURCE_PRIORITY);
		if (val->intval)
			ret |= CFG_INPUT_SOURCE_PRIORITY;
		ret = smb347_write(smb, CFG_VARIOUS_FUNCTION, ret);
		smb347_hw_init(smb);
		if (ret < 0)
			goto priority_fail;
		ret = 0;
priority_fail:
		smb347_set_writable(smb, false);
		break;

	case POWER_SUPPLY_PROP_AUTO_CURRENT_LIMIT:
		smb347_set_writable(smb, true);
		ret = smb347_read(smb, CFG_VARIOUS_FUNCTION);
		if (ret < 0)
			goto aicl_fail;
		ret &= ~(CFG_AUTOMATIC_INPUT_CURRENT_LIMIT);
		if (val->intval)
			ret |= CFG_AUTOMATIC_INPUT_CURRENT_LIMIT;
		ret = smb347_write(smb, CFG_VARIOUS_FUNCTION, ret);
		if (ret < 0)
			goto aicl_fail;
		ret = 0;
aicl_fail:
		smb347_set_writable(smb, false);
		break;
#else

	case POWER_SUPPLY_PROP_AUTO_CURRENT_LIMIT:
		break;
#endif

	default:
		break;
	}

	return ret;
}

#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
static void nxe2000_resume_power_work(struct work_struct *work)
{
	struct nxe2000_power	*nxe2000_power = container_of(work,
		struct nxe2000_power, suspend_on_work.work);
	struct nxe2000_dev		*iodev = nxe2000_power->iodev;

	dev_dbg(nxe2000_power->dev, "Resume Power On\n");

    nxe2000_update(iodev->i2c, NXE2000_REG_LDOEN1, 0x40, 0x40);
}
#endif

#ifdef NXE2000_USED_TIMER_THREAD
static void nxe2000_rdstate_timer_expired(unsigned long data)
{
	struct nxe2000_power	*nxe2000_power = (struct nxe2000_power *)data;
	struct nxe2000_dev		*iodev;

	if (nxe2000_power_suspend_status && !nxe2000_power_resume_status)
	{
		return;
	}

	nxe2000_power_resume_status = 0;

	iodev = nxe2000_power->iodev;

	dev_dbg(nxe2000_power->dev, "Power source changed\n");

	/* Just notify for everything - little harm in overnotifying. */
#if 0
	if (nxe2000_power->have_battery)
		power_supply_changed(&nxe2000_power->battery);
	power_supply_changed(&nxe2000_power->usb);
	power_supply_changed(&nxe2000_power->wall);
#else

	power_supply_changed(&nxe2000_power->battery);
#endif

	if (nxe2000_power->rdstate_repeat_time)
		mod_timer(&nxe2000_power->rdstate_timer, jiffies +
				msecs_to_jiffies(nxe2000_power->rdstate_repeat_time));
}
#else

static irqreturn_t nxe2000_pwr_src_irq(int irq, void *data)
{
	struct nxe2000_power	*nxe2000_power = (struct nxe2000_power *)data;
	struct nxe2000_dev		*iodev;

	if (power_irq_handler_flag)
		goto exit_nxe2000_pwr_src_irq;

	power_irq_handler_flag = 1;

	iodev = nxe2000_power->iodev;

	dev_dbg(nxe2000_power->dev, "Power source changed\n");

	/* Just notify for everything - little harm in overnotifying. */
#if 0
	if (nxe2000_power->have_battery)
		power_supply_changed(&nxe2000_power->battery);
	power_supply_changed(&nxe2000_power->usb);
	power_supply_changed(&nxe2000_power->wall);
#else

	power_supply_changed(&nxe2000_power->battery);
#endif

	power_irq_handler_flag = 0;

exit_nxe2000_pwr_src_irq:
	return IRQ_HANDLED;
}
#endif

#if defined(CONFIG_USB_CONNECT_NXP_DRV)
static void nxe2000_charger_work(struct work_struct *data)
{
//	int                 detect;
	unsigned long       event;
	unsigned            power;
	struct nxe2000_power *nxe2000_power =
		container_of(data, struct nxe2000_power, otg_work);
	static DEFINE_MUTEX(lock);

	event = nxe2000_power->event;
	power = nxe2000_power->max_power;

	mutex_lock(&lock);


#if 0
	if (event != USB_EVENT_NONE)
		isp1704_charger_set_power(nxe2000_power, 1);
#endif

	switch (event) {
    case USB_EVENT_ID:
        break;
	case USB_EVENT_VBUS:
        if (nxe2000_power->gpio_otg_vbus > -1)
        {
            gpio_set_value(nxe2000_power->gpio_otg_vbus, 0);
        }
		break;
	case USB_EVENT_NONE:
        if (nxe2000_power->gpio_otg_vbus > -1)
        {
            gpio_set_value(nxe2000_power->gpio_otg_vbus, 0);
        }
		break;
    case USB_EVENT_CHARGER:
        break;
	case USB_EVENT_ENUMERATED:
		break;
	default:
		goto out;
	}

//	power_supply_changed(&isp->psy);
out:
	mutex_unlock(&lock);
}

static int nxe2000_notifier_call(struct notifier_block *nb,
		unsigned long event, void *power)
{
	struct nxe2000_power *nxe2000_power =
		container_of(nb, struct nxe2000_power, otg_nb);

	nxe2000_power->event = event;

	if (power)
		nxe2000_power->max_power = *((unsigned *)power);

	schedule_work(&nxe2000_power->otg_work);

	return NOTIFY_OK;
}
#endif  /* CONFIG_USB_CONNECT_NXP_DRV */

static void set_gpio_config(struct nxe2000_power *power)
{
#if defined(CONFIG_USB_DWCOTG)
    if ( (power->gpio_otg_usbid > -1)
        && (nxp_soc_gpio_get_io_func(power->gpio_otg_usbid) != nxp_soc_gpio_get_altnum(power->gpio_otg_usbid)) )
    {
        nxp_soc_gpio_set_io_func(power->gpio_otg_usbid, nxp_soc_gpio_get_altnum(power->gpio_otg_usbid));
        nxp_soc_gpio_set_io_dir(power->gpio_otg_usbid, 0);      // input mode
    }

    if ( (power->gpio_otg_vbus > -1)
        && (nxp_soc_gpio_get_io_func(power->gpio_otg_vbus) != nxp_soc_gpio_get_altnum(power->gpio_otg_vbus)) )
    {
        nxp_soc_gpio_set_io_func(power->gpio_otg_vbus, nxp_soc_gpio_get_altnum(power->gpio_otg_vbus));
        nxp_soc_gpio_set_io_dir(power->gpio_otg_vbus, 1);       // output mode
        nxp_soc_gpio_set_out_value(power->gpio_otg_vbus, 0);
    }
#endif

    if ( (power->gpio_pmic_vbus > -1)
        && (nxp_soc_gpio_get_io_func(power->gpio_pmic_vbus) != nxp_soc_gpio_get_altnum(power->gpio_pmic_vbus)) )
    {
        nxp_soc_gpio_set_io_func(power->gpio_pmic_vbus, nxp_soc_gpio_get_altnum(power->gpio_pmic_vbus));
        nxp_soc_gpio_set_io_dir(power->gpio_pmic_vbus, 0);      // input mode
    }

    if ( (power->gpio_pmic_lowbat > -1)
        && (nxp_soc_gpio_get_io_func(power->gpio_pmic_lowbat) != nxp_soc_gpio_get_altnum(power->gpio_pmic_lowbat)) )
    {
        nxp_soc_gpio_set_io_func(power->gpio_pmic_lowbat, nxp_soc_gpio_get_altnum(power->gpio_pmic_lowbat));
        nxp_soc_gpio_set_io_dir(power->gpio_pmic_lowbat, 0);    // input mode
    }

    if ( (power->gpio_eint > -1)
        && (nxp_soc_gpio_get_io_func(power->gpio_eint) != nxp_soc_gpio_get_altnum(power->gpio_eint)) )
    {
        nxp_soc_gpio_set_io_func(power->gpio_eint, nxp_soc_gpio_get_altnum(power->gpio_eint));
        nxp_soc_gpio_set_io_dir(power->gpio_eint, 0);           // input mode
    }
}

static __devinit int nxe2000_power_probe(struct platform_device *pdev)
{
	static char *supply_list[] = { "nxe2000-battery" };
	struct nxe2000_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct nxe2000_pdata *pdata = dev_get_platdata(iodev->dev);
	struct nxe2000_power *power;
	struct power_supply *usb;
	struct power_supply *battery;
	struct power_supply *wall;
	struct i2c_client *i2c;
#ifndef	NXE2000_USED_TIMER_THREAD
	int irq;
#endif
	int ret = 0;
    u8  u8Temp;

	DBGOUT("%s\n", __func__);

	if (!pdata) {
		dev_err(pdev->dev.parent, "No platform init data supplied\n");
		return -ENODEV;
	}

	power = kzalloc(sizeof(struct nxe2000_power), GFP_KERNEL);
	if (!power)
		return -ENOMEM;

#if defined(CONFIG_USB_CONNECT_NXP_DRV)
	/*
	 * REVISIT: using work in order to allow the usb notifications to be
	 * made atomically in the future.
	 */

	power->otg_phy = usb_get_transceiver();
	if (!power->otg_phy)
	{
		pr_err("%s: No USB transceiver found\n", __func__);
	}
	else
	{
#if 1
		INIT_WORK(&power->otg_work, nxe2000_charger_work);

		power->otg_nb.notifier_call = nxe2000_notifier_call;
		ret = usb_register_notifier(power->otg_phy, &power->otg_nb);
		if (ret) {
			pr_err("%s: usb_register_notifier on transceiver %s failed\n",
			       __func__, dev_name(power->otg_phy->dev));
		}
#endif
	}
#endif

	mutex_init(&power->lock);
	power->dev = &pdev->dev;
	power->iodev = iodev;

	/* get platdata */
	memcpy(&power->platdata, pdata, sizeof(struct nxe2000_pdata));
	pdata = &power->platdata;

	platform_set_drvdata(pdev, power);
	i2c = power->iodev->i2c;

	power->have_battery		= pdata->have_battery;
//  power->have_battery		= 1;

	power->gpio_eint		= pdata->gpio_eint;
	power->gpio_otg_usbid	= pdata->gpio_otg_usbid;
	power->gpio_otg_vbus	= pdata->gpio_otg_vbus;
	power->gpio_pmic_vbus	= pdata->gpio_pmic_vbus;
	power->gpio_pmic_lowbat = pdata->gpio_pmic_lowbat;

	power->bat_low_uV		= pdata->bat_low_uV;
	power->bat_max_uV		= pdata->bat_max_uV;
	power->bat_min_uV		= pdata->bat_min_uV;

	power->adp_ilim_current = pdata->adp_ilim_current;
	power->adp_chg_current	= pdata->adp_chg_current;

	power->usb_ilim_current	= pdata->usb_ilim_current;
	power->usb_chg_current	= pdata->usb_chg_current;

#if 0
	for (u8Temp = 0; u8Temp < NXE2000_NUM_BUCK; u8Temp++)
		power->slp_prio_buck[u8Temp]	= pdata->slp_prio_buck[u8Temp];

	for (u8Temp = 0; u8Temp < NXE2000_NUM_LDO; u8Temp++)
		power->slp_prio_ldo[u8Temp]		= pdata->slp_prio_ldo[u8Temp];

	for (u8Temp = 0; u8Temp < NXE2000_NUM_PSO; u8Temp++)
		power->slp_prio_pso[u8Temp]		= pdata->slp_prio_pso[u8Temp];
#endif

    set_gpio_config(power);

#if defined(CONFIG_USB_DWCOTG)
    if (power->gpio_otg_usbid > -1)
        power->otg_dev_mode = gpio_get_value(power->gpio_otg_usbid) ? false : true;
    else
        power->otg_dev_mode = true;
#endif

	usb = &power->usb;
	battery = &power->battery;
	wall = &power->wall;

	snprintf(power->wall_name, sizeof(power->wall_name),
		 "nxe2000-wall");
	snprintf(power->battery_name, sizeof(power->wall_name),
		 "nxe2000-battery");
	snprintf(power->usb_name, sizeof(power->wall_name),
		 "nxe2000-usb");

	/* Setup "End of Charge" */
	/* If EOC value equals 0,
	 * remain value set from bootloader or default value */
	if (pdata->eoc_mA >= 10 && pdata->eoc_mA <= 45) {
		nxe2000_update(i2c, NXE2000_REG_BATSET1,
				(pdata->eoc_mA / 5 - 2) << 5, 0x7 << 5);
	} else if (pdata->eoc_mA == 0) {
		dev_dbg(power->dev,
			"EOC value not set: leave it unchanged.\n");
	} else {
		dev_err(power->dev, "Invalid EOC value\n");
		ret = -EINVAL;
		goto err_kmalloc;
	}

//	power->event_ubc_recheck  = true;
	power->power_src        = 0xFF;

	/* Disable Charger/ADC interrupt */
	ret = nxe2000_clr_bits(i2c, NXE2000_REG_INTEN,
						(1 << NXE2000_POS_INTEN_CHGIREN) | (1 << NXE2000_POS_INTEN_ADCIREN));
	if (ret)
		goto err_kmalloc;


#if (CFG_NXE2000_BAT_CHG_SUPPORT == 1)
	nxe2000_write(i2c, NXE2000_REG_CHGISET,  0);
	nxe2000_write(i2c, NXE2000_REG_REGISET1, 0);
	nxe2000_write(i2c, NXE2000_REG_REGISET2, 0);

	u8Temp	= (0x1 << NXE2000_POS_CHGCTL1_CHGP)
//			| (0x1 << NXE2000_POS_CHGCTL1_VUSBCHGEN)
//			| (0x1 << NXE2000_POS_CHGCTL1_VADPCHGEN)
			| (0x1 << NXE2000_POS_CHGCTL1_JEITAEN)
			| (0x1 << NXE2000_POS_CHGCTL1_SUSPEND);
	nxe2000_write(i2c, NXE2000_REG_CHGCTL1, u8Temp);

	ret = nxe2000_init_battery(power);
	if (ret)
		goto err_kmalloc;
#endif

    /* Set the higher-priority between VADP and VUSB charges. */
#if 0
    u8Temp  = (NXE2000_CHG_PRIO << NXE2000_POS_CHGCTL1_CHGP)
			| (0x1 << NXE2000_POS_CHGCTL1_SUSPEND)
			| (0x1 << NXE2000_POS_CHGCTL1_VUSBCHGEN)
			| (0x1 << NXE2000_POS_CHGCTL1_VADPCHGEN);
#else
	u8Temp  = (NXE2000_CHG_PRIO << NXE2000_POS_CHGCTL1_CHGP)
			| (0x1 << NXE2000_POS_CHGCTL1_SUSPEND)
			| (0x1 << NXE2000_CHG_PRIO);
#endif
	nxe2000_write(i2c, NXE2000_REG_CHGCTL1, u8Temp);

//	nxe2000_write(i2c, 0xDC, 0xFF);
//	nxe2000_write(i2c, 0xDE, 0xFF);

	nxe2000_set_chg_current(i2c, NXE2000_PWR_SRC_BAT, power->usb_chg_current, 0);
	nxe2000_set_chg_current(i2c, NXE2000_PWR_SRC_ADP, power->adp_ilim_current, 0);
	nxe2000_set_chg_current(i2c, NXE2000_PWR_SRC_USB, power->usb_ilim_current, 0);

	/* Feul Gauge Enable */
#if 0
	if (power->first_pwon)
	{
		u8Temp  = (0x1 << NXE2000_POS_FG_CTRL_SRST1)
				| (0x1 << NXE2000_POS_FG_CTRL_FG_ACC)
				| (0x1 << NXE2000_POS_FG_CTRL_FG_EN);
	}
	else
	{
		u8Temp  = (0x1 << NXE2000_POS_FG_CTRL_FG_ACC)
				| (0x1 << NXE2000_POS_FG_CTRL_FG_EN);
	}
	nxe2000_write(i2c, NXE2000_REG_FG_CTRL, u8Temp);
#endif

	/* Setup Charge Restart Level */
	switch (pdata->restart) {
	case 100:
		nxe2000_update(i2c, NXE2000_REG_BATSET1, 0x1 << 3, 0x3 << 3);
		break;
	case 150:
		nxe2000_update(i2c, NXE2000_REG_BATSET1, 0x0 << 3, 0x3 << 3);
		break;
	case 200:
		nxe2000_update(i2c, NXE2000_REG_BATSET1, 0x2 << 3, 0x3 << 3);
		break;
	case -1:
		nxe2000_update(i2c, NXE2000_REG_BATSET1, 0x3 << 3, 0x3 << 3);
		break;
	case 0:
		dev_dbg(power->dev,
			"Restart Level not set: leave it unchanged.\n");
		break;
	default:
		dev_err(power->dev, "Invalid Restart Level\n");
		ret = -EINVAL;
		goto err_kmalloc;
	}

	/* Setup Charge Full Timeout */
	switch (pdata->timeout) {
	case 5:
		nxe2000_update(i2c, NXE2000_REG_TT_FULL_H, 0x0 << 4, 0x3 << 4);
		break;
	case 6:
		nxe2000_update(i2c, NXE2000_REG_TT_FULL_H, 0x1 << 4, 0x3 << 4);
		break;
	case 7:
		nxe2000_update(i2c, NXE2000_REG_TT_FULL_H, 0x2 << 4, 0x3 << 4);
		break;
	case -1:
		nxe2000_update(i2c, NXE2000_REG_TT_FULL_H, 0x3 << 4, 0x3 << 4);
		break;
	case 0:
		dev_dbg(power->dev,
			"Full Timeout not set: leave it unchanged.\n");
		break;
	default:
		dev_err(power->dev, "Invalid Full Timeout value\n");
		ret = -EINVAL;
		goto err_kmalloc;
	}

	/* We ignore configuration failures since we can still read back
	 * the status without enabling the charger.
	 */

#if 1
#if 1
	wall->name = power->wall_name;
	wall->type = POWER_SUPPLY_TYPE_MAINS;
	wall->supplied_to = supply_list;
	wall->num_supplicants = ARRAY_SIZE(supply_list);
	wall->properties = nxe2000_wall_props;
	wall->num_properties = ARRAY_SIZE(nxe2000_wall_props);
	wall->get_property = nxe2000_wall_get_prop;
	ret = power_supply_register(&pdev->dev, wall);
	if (ret)
		goto err_kmalloc;

	usb->name = power->usb_name,
	usb->type = POWER_SUPPLY_TYPE_USB;
	usb->supplied_to = supply_list;
	usb->num_supplicants = ARRAY_SIZE(supply_list);
	usb->properties = nxe2000_usb_props;
	usb->num_properties = ARRAY_SIZE(nxe2000_usb_props);
	usb->get_property = nxe2000_usb_get_prop;
	ret = power_supply_register(&pdev->dev, usb);
	if (ret)
		goto err_wall;
#endif

	if (power->have_battery) {
		battery->name = power->battery_name;
		battery->type = POWER_SUPPLY_TYPE_BATTERY;
		battery->properties = nxe2000_bat_props;
		battery->num_properties = ARRAY_SIZE(nxe2000_bat_props);
		battery->get_property = nxe2000_bat_get_prop;
#if 0
		battery->set_property = nxe2000_bat_set_prop;
		battery->property_is_writeable = nxe2000_bat_prop_is_writeable;
        battery->external_power_changed	= power_supply_changed;
		battery->use_for_apm = 1;
#endif
		ret = power_supply_register(&pdev->dev, battery);
		if (ret)
			goto err_usb;
	}
#else

	battery->name = power->battery_name;
	battery->type = POWER_SUPPLY_TYPE_BATTERY;
	battery->properties = nxe2000_bat_props;
	battery->num_properties = ARRAY_SIZE(nxe2000_bat_props);
	battery->get_property = nxe2000_bat_get_prop;
	battery->use_for_apm = 1;
	ret = power_supply_register(&pdev->dev, battery);
	if (ret)
		goto err_kmalloc;
#endif

	nxe2000_power_suspend_status = 0;
	nxe2000_power_resume_status  = 0;
	nxe2000_power_resume_check   = 0;
	nxe2000_power_lowbat         = 0;
    nxe2000_power_lowbat_resume_count   = 0;

#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
	power->suspend_on_wqueue
		= create_singlethread_workqueue("nxe2000_suspend_work");
	INIT_DELAYED_WORK_DEFERRABLE(&power->suspend_on_work,
						nxe2000_resume_power_work);

	queue_delayed_work(power->suspend_on_wqueue,
						&power->suspend_on_work,
						HZ);
#endif

#ifdef	NXE2000_USED_TIMER_THREAD
	/* Initilialize safety timer */
	init_timer(&power->rdstate_timer);
	power->rdstate_timer.function	= nxe2000_rdstate_timer_expired;
	power->rdstate_timer.data		= (unsigned long) power;
	power->rdstate_timer.expires	= jiffies + msecs_to_jiffies(1000);
	power->rdstate_repeat_time		= pdata->rdstate_periodic;

	add_timer(&power->rdstate_timer);
#else

#ifdef	NXE2000_RDSTATE_ONLY
	irq = iodev->irq_base + NXE2000_IRQ_CHGCTRL_RDSTATESHIFT;
#else
	irq = iodev->irq_base + NXE2000_IRQ_CHGCTRL_VADPDETS;
#endif
	ret = request_threaded_irq(irq, NULL, nxe2000_pwr_src_irq,
				   IRQF_TRIGGER_RISING, "Power source",
				   power);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request PWR SRC IRQ %d: %d\n",
			irq, ret);
		goto err_battery;
	}

#ifndef	NXE2000_RDSTATE_ONLY
	irq = iodev->irq_base + NXE2000_IRQ_CHGCTRL_VUSBDETS;
	ret = request_threaded_irq(irq, NULL, nxe2000_pwr_src_irq,
				   IRQF_TRIGGER_RISING, "Power source",
				   power);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request PWR SRC IRQ %d: %d\n",
			irq, ret);
		goto err_adp_irq;
	}
#endif
#endif	// #ifdef NXE2000_USED_TIMER_THREAD

	nxe2000_config_battery(iodev);

	return 0;
#ifndef	NXE2000_USED_TIMER_THREAD
err_adp_irq:
#ifdef	NXE2000_RDSTATE_ONLY
	irq = iodev->irq_base + NXE2000_IRQ_CHGCTRL_RDSTATESHIFT;
#else
	irq = iodev->irq_base + NXE2000_IRQ_CHGCTRL_VADPDETS;
#endif
	free_irq(irq, power);
err_battery:
#endif	// #ifdef NXE2000_USED_TIMER_THREAD
#if 1
	if (power->have_battery)
		power_supply_unregister(battery);
err_usb:
	power_supply_unregister(usb);
err_wall:
	power_supply_unregister(wall);
#else

	power_supply_unregister(battery);
#endif
err_kmalloc:
	kfree(power);
	return ret;
}

static int __devexit nxe2000_power_remove(struct platform_device *pdev)
{
	struct nxe2000_power *charger = platform_get_drvdata(pdev);

	DBGOUT("%s\n", __func__);

	power_supply_unregister(&charger->battery);
	kfree(charger);

	return 0;
}

#ifdef CONFIG_PM
#if 0
static int nxe2000_battery_suspend(struct device *dev)
{
	struct nxe2000_battery_info *info = dev_get_drvdata(dev);
	uint8_t val;
	int ret;
	int err;
	int cc_cap = 0;
	bool is_charging = true;

#ifdef ENABLE_MASKING_INTERRUPT_IN_SLEEP
	nxe2000_clr_bits(dev->parent, RICOH61x_INTC_INTEN, CHG_INT);
#endif

	if (g_fg_on_mode
		 && (info->soca->status == RICOH61x_SOCA_STABLE)) {
		err = nxe2000_write(info->dev->parent, PSWR_REG, 0x7f);
		if (err < 0)
			dev_err(info->dev, "Error in writing PSWR_REG\n");
		 g_soc = 0x7F;
		info->soca->suspend_soc = (info->soca->displayed_soc + 50)/100;
	} else {
		if (info->soca->displayed_soc < 0) {
			val = 0;
		} else {
			val = (info->soca->displayed_soc + 50)/100;
			val &= 0x7f;
		}
		ret = nxe2000_write(info->dev->parent, PSWR_REG, val);
		if (ret < 0)
			dev_err(info->dev, "Error in writing PSWR_REG\n");

		g_soc = (info->soca->displayed_soc + 50)/100;
		info->soca->suspend_soc = (info->soca->displayed_soc + 50)/100;

		ret = calc_capacity_in_period(info, &cc_cap, &is_charging);
		if (ret < 0)
			dev_err(info->dev, "Read cc_sum Error !!-----\n");
	}

	if (info->soca->status == RICOH61x_SOCA_STABLE
		|| info->soca->status == RICOH61x_SOCA_FULL)
		info->soca->status = RICOH61x_SOCA_DISP;

	disable_irq(charger_irq + RICOH61x_IRQ_FONCHGINT);
	
#ifdef ENABLE_MASKING_INTERRUPT_IN_SLEEP	
	disable_irq(charger_irq + RICOH61x_IRQ_FCHGCMPINT);
	disable_irq(charger_irq + RICOH61x_IRQ_FVUSBDETSINT);
	disable_irq(charger_irq + RICOH61x_IRQ_FVADPDETSINT);
#endif	
#ifdef ENABLE_LOW_BATTERY_DETECTION
	/*disable_irq(charger_irq + RICOH61x_IRQ_VSYSLIR);*/
#endif

	flush_delayed_work(&info->monitor_work);
	flush_delayed_work(&info->displayed_work);
	flush_delayed_work(&info->charge_stable_work);
	flush_delayed_work(&info->get_charge_work);
	flush_delayed_work(&info->changed_work);
#ifdef ENABLE_LOW_BATTERY_DETECTION
	flush_delayed_work(&info->low_battery_work);
#endif
#ifdef ENABLE_FACTORY_MODE
	flush_delayed_work(&info->factory_mode_work);
#endif

/*	flush_work(&info->irq_work); */


	return 0;
}

static int nxe2000_battery_resume(struct device *dev)
{
	struct nxe2000_battery_info *info = dev_get_drvdata(dev);
	uint8_t val;
	int ret;
	int displayed_soc_temp;
	int cc_cap = 0;
	bool is_charging = true;
	int i;

#ifdef ENABLE_MASKING_INTERRUPT_IN_SLEEP
	nxe2000_set_bits(dev->parent, RICOH61x_INTC_INTEN, CHG_INT);
#endif

	if (info->entry_factory_mode) {
		info->soca->displayed_soc = -EINVAL;
	} else if (RICOH61x_SOCA_ZERO == info->soca->status) {
		if (calc_ocv(info) > get_OCV_voltage(info, 0)) {
			ret = nxe2000_read(info->dev->parent, PSWR_REG, &val);
			val &= 0x7f;
			info->soca->soc = val * 100;
			if (ret < 0) {
				dev_err(info->dev,
					 "Error in reading PSWR_REG %d\n", ret);
				info->soca->soc
					 = calc_capacity(info) * 100;
			}

			ret = calc_capacity_in_period(info, &cc_cap,
								 &is_charging);
			if (ret < 0)
				dev_err(info->dev, "Read cc_sum Error !!-----\n");

			info->soca->cc_delta
				 = (is_charging == true) ? cc_cap : -cc_cap;

			displayed_soc_temp
				 = info->soca->soc + info->soca->cc_delta;
			if (displayed_soc_temp < 0)
				displayed_soc_temp = 0;
			
			displayed_soc_temp = min(10000, displayed_soc_temp);
			displayed_soc_temp = max(0, displayed_soc_temp);
			info->soca->displayed_soc = displayed_soc_temp;

			ret = nxe2000_write(info->dev->parent,
							 FG_CTRL_REG, 0x51);
			if (ret < 0)
				dev_err(info->dev, "Error in writing the control register\n");
			info->soca->ready_fg = 0;
			info->soca->status = RICOH61x_SOCA_FG_RESET;

		} else
			info->soca->displayed_soc = 0;
	} else {
		info->soca->soc = info->soca->suspend_soc * 100;

		ret = calc_capacity_in_period(info, &cc_cap, &is_charging);
		if (ret < 0)
			dev_err(info->dev, "Read cc_sum Error !!-----\n");

		info->soca->cc_delta = (is_charging == true) ? cc_cap : -cc_cap;

		displayed_soc_temp = info->soca->soc + info->soca->cc_delta;
		if (displayed_soc_temp < 0)
				displayed_soc_temp = 0;
		
		displayed_soc_temp = min(10000, displayed_soc_temp);
		displayed_soc_temp = max(0, displayed_soc_temp);

		check_charge_status_2(info, displayed_soc_temp);

		if (RICOH61x_SOCA_DISP == info->soca->status) {
			info->soca->last_soc = calc_capacity(info) * 100;
			info->soca->soc_delta = 0;
		}
	}
	info->soca->update_count = 0;

	ret = measure_vbatt_FG(info, &info->soca->Vbat_ave);
	ret = measure_vsys_ADC(info, &info->soca->Vsys_ave);
	ret = measure_Ibatt_FG(info, &info->soca->Ibat_ave);

	power_supply_changed(&info->battery);
	queue_delayed_work(info->monitor_wqueue, &info->displayed_work, HZ);

	if (RICOH61x_SOCA_UNSTABLE == info->soca->status) {
		info->soca->stable_count = 10;
		queue_delayed_work(info->monitor_wqueue,
					 &info->charge_stable_work,
					 RICOH61x_FG_STABLE_TIME*HZ/10);
	} else if (RICOH61x_SOCA_FG_RESET == info->soca->status) {
		info->soca->stable_count = 1;

		for (i = 0; i < 3; i = i+1)
			info->soca->reset_soc[i] = 0;
		info->soca->reset_count = 0;

		queue_delayed_work(info->monitor_wqueue,
					 &info->charge_stable_work,
					 RICOH61x_FG_RESET_TIME*HZ);
	}

	queue_delayed_work(info->monitor_wqueue, &info->monitor_work,
						 info->monitor_time);

	info->soca->chg_count = 0;
	queue_delayed_work(info->monitor_wqueue, &info->get_charge_work,
					 RICOH61x_CHARGE_RESUME_TIME * HZ);

	enable_irq(charger_irq + RICOH61x_IRQ_FONCHGINT);
#ifdef ENABLE_MASKING_INTERRUPT_IN_SLEEP
	enable_irq(charger_irq + RICOH61x_IRQ_FCHGCMPINT);
	enable_irq(charger_irq + RICOH61x_IRQ_FVUSBDETSINT);
	enable_irq(charger_irq + RICOH61x_IRQ_FVADPDETSINT);
#endif
#ifdef ENABLE_LOW_BATTERY_DETECTION
	/*enable_irq(charger_irq + RICOH61x_IRQ_VSYSLIR);*/
#endif
	return 0;
}
#else

static int nxe2000_battery_suspend(struct device *dev)
{
	struct nxe2000_power *power = dev_get_drvdata(dev);
    struct i2c_client    *i2c   = power->iodev->i2c;
    u8 val;

	while (nxe2000_power_resume_check)
		msleep(5);

	nxe2000_power_suspend_status = 1;
	nxe2000_power_lowbat         = 2;

	del_timer_sync(&power->rdstate_timer);

#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
	flush_delayed_work(&power->suspend_on_work);

//	if (power->slp_prio_ldo[7] == 0xF)
//	{
//		nxe2000_update(iodev->i2c, NXE2000_REG_LDOEN1, 0x00, 0x40);
//	}
#endif

	if ( (power->bat_low_uV > -1) && (!nxe2000_power_lowbat_resume_count) )
	{
		/* Setting to threshold of battery voltage. */
		val = (((power->bat_low_uV / 1000) * 255) / 5000) & 0xFF;
		nxe2000_write(i2c, NXE2000_REG_VBATTHL, val);
//		nxe2000_write(i2c, NXE2000_REG_VBATTHH, val);

		val = (NXE2000_POS_ADCCNT3_ADRQ_AUTO | (1 << NXE2000_POS_ADCCNT3_AVE));
		nxe2000_write(i2c, NXE2000_REG_ADCCNT3, val);

		val = (1 << NXE2000_POS_ADC_VBAT);
		nxe2000_write(i2c, NXE2000_REG_ADCCNT1, val);

        /* Clear to interrupt pending. */
		nxe2000_write(i2c, NXE2000_REG_IR_ADC1, 0x00);
		nxe2000_write(i2c, NXE2000_REG_IR_ADC2, 0x00);
		nxe2000_write(i2c, NXE2000_REG_IR_ADC3, 0x00);

        /* Setting to interrupt mode. */
		val = (1 << NXE2000_POS_INTEN_ADCIREN);
		nxe2000_write(i2c, NXE2000_REG_INTEN, val);

		val = (1 << NXE2000_POS_ADC_VBAT);
		nxe2000_write(i2c, NXE2000_REG_EN_ADCIR1, val);
//		nxe2000_write(i2c, NXE2000_REG_EN_ADCIR2, val);

		nxe2000_write(i2c, NXE2000_REG_EN_ADCIR3, 0x01);
	}

#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
	if (power->slp_prio_ldo[7] == 0xF)
	{
		nxe2000_update(i2c, NXE2000_REG_LDOEN1, 0x00, 0x80);
	}
#endif

	return 0;
}

static int nxe2000_battery_resume(struct device *dev)
{
	struct nxe2000_power *power = dev_get_drvdata(dev);
	struct i2c_client    *i2c   = power->iodev->i2c;
    int ret;
    u8  val;

power->event_ubc_recheck = true;

	ret = nxe2000_read(i2c, NXE2000_REG_IR_ADC3, &val);
    if (ret)
        printk(KERN_INFO "Error : Do not read NXE2000_REG_IR_ADC3 register\n");
	ret = nxe2000_read(i2c, NXE2000_REG_IR_ADC1, &val);
    if (ret)
        printk(KERN_INFO "Error : Do not read NXE2000_REG_IR_ADC1 register\n");
	val &= (1 << NXE2000_POS_ADC_VBAT);

	if ( val && !nxe2000_power_lowbat_resume_count )
	{
		/* Interrupt disable. */
		nxe2000_write(i2c,  NXE2000_REG_INTEN,      0x00);

		nxe2000_write(i2c,  NXE2000_REG_EN_ADCIR3,  0x00);
		nxe2000_write(i2c,  NXE2000_REG_EN_ADCIR1,  0x00);
		nxe2000_write(i2c,  NXE2000_REG_EN_ADCIR2,  0x00);

		nxe2000_write(i2c,  NXE2000_REG_ADCCNT3,    0x00);
		nxe2000_write(i2c,  NXE2000_REG_ADCCNT1,    0x00);

		/* Clear to interrupt pending. */
#if 0
		nxe2000_write(i2c,  NXE2000_REG_IR_ADC1,    0x00);
		nxe2000_write(i2c,  NXE2000_REG_IR_ADC2,    0x00);
		nxe2000_write(i2c,  NXE2000_REG_IR_ADC3,    0x00);
#endif

//		nxe2000_power_lowbat_resume_count++;
	}

//	if (gpio_get_value(power->gpio_pmic_vbus) || !gpio_get_value(power->gpio_pmic_lowbat))
	{
		power_supply_changed(&power->battery);
//		power_supply_changed(&power->usb);
//		power_supply_changed(&power->wall);
		nxe2000_power_resume_status = 1;
		nxe2000_power_resume_check  = 1;
	}

	nxe2000_power_suspend_status = 0;

	if (!timer_pending(&power->rdstate_timer))
		add_timer(&power->rdstate_timer);
	else
        mod_timer(&power->rdstate_timer,
					jiffies + msecs_to_jiffies(100));

#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
	if (power->slp_prio_ldo[7] == 0xF)
	{
		queue_delayed_work(power->suspend_on_wqueue,
					&power->suspend_on_work,
				 	jiffies + msecs_to_jiffies(100));
	}
#endif

//	nxe2000_power_suspend_status = 0;

	return 0;
}
#endif

static const struct dev_pm_ops nxe2000_battery_pm_ops = {
	.suspend	= nxe2000_battery_suspend,
	.resume		= nxe2000_battery_resume,
};
#endif

static const struct platform_device_id nxe2000_power_id[] = {
	{ "nxe2000-battery", TYPE_NXE2000 },
	{ }
};

static struct platform_driver nxe2000_power_driver = {
	.driver = {
		.name = "nxe2000-power",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &nxe2000_battery_pm_ops,
#endif
	},
	.probe = nxe2000_power_probe,
	.remove = __devexit_p(nxe2000_power_remove),
	.id_table = nxe2000_power_id,
};

#if 1
module_platform_driver(nxe2000_power_driver);
#else
static int __init nxe2000_battery_init(void)
{
	return platform_driver_register(&nxe2000_power_driver);
}
subsys_initcall(nxe2000_battery_init);

static void __exit nxe2000_battery_cleanup(void)
{
	platform_driver_unregister(&nxe2000_power_driver);
}
module_exit(nxe2000_battery_cleanup);
#endif

MODULE_DESCRIPTION("NXE2000 battery control driver");
MODULE_AUTHOR("Bongkwan Kook <kook@nexell.co.kr>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nxe2000-battery");
