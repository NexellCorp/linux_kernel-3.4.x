/*
 * nxe1100_charger.c - Power supply consumer driver for the NXE1100
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

/* nexell soc headers */
#include <nxe1100.h>
#include <nxe1100-private.h>

/*
 * Debug
 */
#if (0)
#define DBGOUT(msg...)		{ printk("nxe1100-charger: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define NXE1100_USED_TIMER_THREAD
#define NXE1100_RDSTATE_ONLY

#define NXE1100_TIMER_THREAD_PERIODIC	(10000)	/* milisecond	*/
//#define NXE1100_FULL_CAP	(2250)
#define NXE1100_FULL_CAP	(2500)

struct nxe1100_power {
	struct device				*dev;
	struct nxe1100_dev			*iodev;
	struct nxe1100_pdata		platdata;
	struct power_supply			wall;
	struct power_supply			usb;
	struct power_supply			battery;
#ifdef NXE1100_USED_TIMER_THREAD
	struct timer_list			rdstate_timer;
	u32							rdstate_repeat_time;
#endif
	bool		wall_online;
	bool		usb_online;
	char		wall_name[20];
	char		usb_name[20];
	char		battery_name[20];
	bool		have_battery;
};

static enum power_supply_property nxe1100_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS, 			/* "FULL" or "NOT FULL" only. */
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_PRESENT, 			/* the presence of battery */
	POWER_SUPPLY_PROP_ONLINE, 			/* charger is active or not */
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
};

struct current_map_desc {
	int min;
	int max;
	int step;
};

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
	[NXE1100_PWR_SRC_BAT] = &chg_current_map_desc,			/* Charge current */
	[NXE1100_PWR_SRC_ADP] = &adp_ilim_current_map_desc,		/* ADP limit current */
	[NXE1100_PWR_SRC_USB] = &usb_ilim_current_map_desc,		/* USB limit current */
};

static u8 nxe1100_get_current_proper_val(
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

static int nxe1100_set_chg_current(struct i2c_client *i2c, enum nxe1100_pwr_src supply, u32 uA)
{
	const struct current_map_desc *desc;
	int min_mA, max_mA;
	u8	value;

	desc	= chg_current_map[supply];

	value	= (uA / 100000);
	min_mA	= value * 100;
	max_mA	= (value + 1) * 100;

	value = nxe1100_get_current_proper_val(desc, min_mA, max_mA);

	switch (supply)
	{
	case NXE1100_PWR_SRC_BAT:
		nxe1100_update_reg(i2c, NXE1100_REG_CHGISET, value, NXE1100_VAL_CHGISET_ICHG_MASK);
		break;
	case NXE1100_PWR_SRC_ADP:
		nxe1100_write_reg(i2c, NXE1100_REG_REGISET1, value);
		break;
	case NXE1100_PWR_SRC_USB:
		nxe1100_write_reg(i2c, NXE1100_REG_REGISET2, value);
		break;
	}

	return 0;
}

static int nxe1100_power_check_online(u8 chg_state, enum nxe1100_pwr_src supply,
				     union power_supply_propval *val)
{
	u8	value;

	val->intval = 0;

	switch (supply)
	{
	case NXE1100_PWR_SRC_BAT:
		if (chg_state != NXE1100_VAL_CHGSTATE_NO_BATT && chg_state != NXE1100_VAL_CHGSTATE_NO_BATT2)
			val->intval = 1;
		break;
	case NXE1100_PWR_SRC_ADP:
	case NXE1100_PWR_SRC_USB:
		value = (chg_state >> NXE1100_POS_CHGSTATE_USEADP) & (1 << supply);
		if (value)
			val->intval = 1;
		break;
	}

	return 0;
}

static void nxe1100_config_battery(struct nxe1100_dev *iodev)
{
	struct nxe1100_pdata *nxe1100_pdata = iodev->dev->platform_data;

	if (!nxe1100_pdata) {
		dev_warn(iodev->dev,
			 "No battery charger configuration\n");
		return;
	}

#ifndef	NXE1100_USED_TIMER_THREAD
#ifdef	NXE1100_RDSTATE_ONLY
	nxe1100_write_reg(iodev->i2c, NXE1100_REG_CHGCTL_IRFMASK,	0x3F);
#else
	nxe1100_write_reg(iodev->i2c, NXE1100_REG_CHGCTL_IRFMASK,	0x00);
	nxe1100_write_reg(iodev->i2c, NXE1100_REG_CHGSTAT_IRFMASK1,	0xF0);
	nxe1100_write_reg(iodev->i2c, NXE1100_REG_CHGSTAT_IRFMASK2,	0xF0);
	nxe1100_write_reg(iodev->i2c, NXE1100_REG_CHGERR_IRFMASK,	0x00);
#endif
#endif
}

static int nxe1100_bat_check_status(u8 chg_state, int *status)
{
	switch (chg_state & NXE1100_POS_CHGSTATE_RDSTATE_MASK) {
	case NXE1100_VAL_CHGSTATE_CHG_OFF:
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case NXE1100_VAL_CHGSTATE_CHG_CMP:
		*status = POWER_SUPPLY_STATUS_FULL;
		break;
	case NXE1100_VAL_CHGSTATE_NO_BATT2:
	case NXE1100_VAL_CHGSTATE_DIE_SHUDN:
	case NXE1100_VAL_CHGSTATE_DIE_ERR:
	case NXE1100_VAL_CHGSTATE_NO_BATT:
	case NXE1100_VAL_CHGSTATE_BATT_ERR:
//	case NXE1100_VAL_CHGSTATE_CHG_OFF:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case NXE1100_VAL_CHGSTATE_TRICKLE_CHG:
	case NXE1100_VAL_CHGSTATE_RAPID_CHG:
		*status = POWER_SUPPLY_STATUS_CHARGING;
		break;

	default:
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

static int nxe1100_bat_check_type(u8 chg_state, int *type)
{
	switch (chg_state & NXE1100_POS_CHGSTATE_RDSTATE_MASK) {
	case NXE1100_VAL_CHGSTATE_TRICKLE_CHG:
		*type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case NXE1100_VAL_CHGSTATE_RAPID_CHG:
		*type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	default:
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	}

	return 0;
}

#if 0
static int nxe1100_battery_check_health(struct i2c_client *i2c, int *health)
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

static int nxe1100_power_read_voltage(struct i2c_client *i2c,
				     union power_supply_propval *val)
{
	u8  value[2];
	int	ret;

	val->intval = 0;

	ret = nxe1100_bulk_read(i2c, NXE1100_REG_VOLTAGE_1, 2, value);
	if (ret)
		goto exit_read_voltage;

	val->intval = (value[0] << 8) | value[1];	/* unit is mA */

exit_read_voltage:
	return ret;
}


/*********************************************************************
 *		WALL Power
 *********************************************************************/
static int nxe1100_wall_get_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct nxe1100_power	*nxe1100_power = dev_get_drvdata(psy->dev->parent);
	struct i2c_client		*i2c = nxe1100_power->iodev->i2c;
	int	ret = 0;
	u8	chg_state;

	chg_state = nxe1100_power->iodev->status_regs[NXE1100_IRQGRP_FG_INT];

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = nxe1100_power_check_online(chg_state, NXE1100_PWR_SRC_ADP, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = nxe1100_power_read_voltage(i2c, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property nxe1100_wall_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/*********************************************************************
 *		USB Power
 *********************************************************************/
static int nxe1100_usb_get_prop(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct nxe1100_power	*nxe1100_power = dev_get_drvdata(psy->dev->parent);
	struct i2c_client		*i2c = nxe1100_power->iodev->i2c;
	int	ret = 0;
	u8	chg_state;

	chg_state = nxe1100_power->iodev->status_regs[NXE1100_IRQGRP_FG_INT];

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = nxe1100_power_check_online(chg_state, NXE1100_PWR_SRC_USB, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = nxe1100_power_read_voltage(i2c, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property nxe1100_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/*********************************************************************
 *		Battery properties
 *********************************************************************/

static int nxe1100_get_bat_time2empty(struct i2c_client *i2c,
				     union power_supply_propval *val)
{
	u8  value[2];
	int	ret;

	val->intval = 0;

	ret = nxe1100_bulk_read(i2c, NXE1100_REG_TT_EMPTY_H, 2, value);
	if (ret)
		goto exit_get_time2empty;

	val->intval = (value[0] << 8) | value[1];	/* unit is minute */

exit_get_time2empty:
	return ret;
}

static int nxe1100_get_bat_time2full(struct i2c_client *i2c,
				     union power_supply_propval *val)
{
	u8  value[2];
	int	ret;

	val->intval = 0;

	ret = nxe1100_bulk_read(i2c, NXE1100_REG_TT_FULL_H, 2, value);
	if (ret)
		goto exit_get_time2full;

	val->intval = (value[0] << 8) | value[1];	/* unit is minute */

exit_get_time2full:
	return ret;
}

static int nxe1100_get_bat_temp(struct i2c_client *i2c,
				     union power_supply_propval *val)
{
	u8  value[2];
	int	data;
	int ret;

	val->intval = 0;

	ret = nxe1100_bulk_read(i2c, NXE1100_REG_TEMP_1, 2, value);
	if (ret)
		goto exit_get_temp;

	data = (value[0] << 8) | value[1];	/*	unit : 0.0625¡ÆC */
	val->intval = (data * 625) / 10000;

exit_get_temp:
	return ret;
}

static int nxe1100_bat_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct nxe1100_power	*nxe1100_power = dev_get_drvdata(psy->dev->parent);
	struct nxe1100_pdata	*pdata = &nxe1100_power->platdata;
	struct i2c_client		*i2c = nxe1100_power->iodev->i2c;
	int	ret;
	u8	reg, chg_state;

	DBGOUT("%s\n", __func__);

	chg_state = nxe1100_power->iodev->status_regs[NXE1100_IRQGRP_FG_INT];

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
#ifdef	NXE1100_USED_TIMER_THREAD
		ret = nxe1100_read_reg(i2c, NXE1100_REG_CHGSTATE,
				&chg_state);
		if (ret) {
			val->intval = 0;
		}
		nxe1100_power->iodev->status_regs[NXE1100_IRQGRP_FG_INT] = chg_state;
#endif
		ret = nxe1100_bat_check_status(chg_state, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = nxe1100_read_reg(i2c, NXE1100_REG_CHGERR_MONI, &reg);
		if (ret)
			return ret;
		if (reg & (NXE1100_POS_CHGERR_MONI_DIEERRINT | NXE1100_POS_CHGERR_MONI_DIEOFFINT))
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:	/* POWER_SUPPLY_STATUS_UNKNOWN, CHARGING, DISCHARGING, NOT_CHARGING, FULL */
		if (chg_state & (NXE1100_VAL_CHGSTATE_NO_BATT2 | NXE1100_VAL_CHGSTATE_NO_BATT))
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_ONLINE:	/* 0: OFF Line, 1: ON Line  */
		ret = nxe1100_power_check_online(chg_state, NXE1100_PWR_SRC_BAT, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = nxe1100_power_read_voltage(i2c, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = pdata->batt_volt_max / 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = pdata->batt_volt_min / 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = nxe1100_bat_check_type(chg_state, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (chg_state & (1 << NXE1100_POS_CHGSTATE_USEADP))
			val->intval = (pdata->adp_ilim_current / 1000);
		else
			val->intval = (pdata->usb_ilim_current / 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = nxe1100_read_reg(i2c, NXE1100_REG_CHGISET, &reg);
		if (ret)
			val->intval = 0;
		else
			val->intval = ((reg & NXE1100_VAL_CHGISET_ICHG_MASK) + 1) * 100;	/* unit is mA */
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if ((chg_state & NXE1100_POS_CHGSTATE_RDSTATE_MASK) == NXE1100_VAL_CHGSTATE_CHG_CMP)
		{
			val->intval = 100;			/* unit is % */
		}
		else
		{
			ret = nxe1100_read_reg(i2c, NXE1100_REG_SOC, &reg);
			if (ret)
				val->intval = 0;
			else
				val->intval = reg;		/* unit is % */
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = pdata->batt_cap_level ? pdata->batt_cap_level : POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = nxe1100_get_bat_temp(i2c, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = nxe1100_get_bat_time2empty(i2c, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = nxe1100_get_bat_time2full(i2c, val);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "NXE1100";
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Nexell";
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#ifdef NXE1100_USED_TIMER_THREAD
static void nxe1100_rdstate_timer_expired(unsigned long data)
{
	struct nxe1100_power	*nxe1100_power = (struct nxe1100_power *)data;
	struct nxe1100_dev		*iodev;

	iodev = nxe1100_power->iodev;

	dev_dbg(nxe1100_power->dev, "Power source changed\n");

	/* Just notify for everything - little harm in overnotifying. */
	if (nxe1100_power->have_battery)
		power_supply_changed(&nxe1100_power->battery);
	power_supply_changed(&nxe1100_power->usb);
	power_supply_changed(&nxe1100_power->wall);

	if (nxe1100_power->rdstate_repeat_time)
		mod_timer(&nxe1100_power->rdstate_timer, jiffies +
				msecs_to_jiffies(nxe1100_power->rdstate_repeat_time));
}
#else

static irqreturn_t nxe1100_pwr_src_irq(int irq, void *data)
{
	struct nxe1100_power	*nxe1100_power = (struct nxe1100_power *)data;
	struct nxe1100_dev		*iodev;
	u8	fg_ctrl;

	if (power_irq_handler_flag)
		goto exit_nxe1100_pwr_src_irq;

	power_irq_handler_flag = 1;

	iodev = nxe1100_power->iodev;

	nxe1100_read_reg(iodev->i2c, NXE1100_REG_FG_CTRL, &fg_ctrl);

	dev_dbg(nxe1100_power->dev, "Power source changed\n");

	/* Just notify for everything - little harm in overnotifying. */
	if (nxe1100_power->have_battery)
		power_supply_changed(&nxe1100_power->battery);
	power_supply_changed(&nxe1100_power->usb);
	power_supply_changed(&nxe1100_power->wall);

	power_irq_handler_flag = 0;

exit_nxe1100_pwr_src_irq:
	return IRQ_HANDLED;
}
#endif

static __devinit int nxe1100_power_probe(struct platform_device *pdev)
{
	struct nxe1100_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct nxe1100_pdata *pdata = dev_get_platdata(iodev->dev);
	struct nxe1100_power *power;
	struct power_supply *usb;
	struct power_supply *battery;
	struct power_supply *wall;
	struct i2c_client *i2c;
#ifndef	NXE1100_USED_TIMER_THREAD
	int irq
#endif
	int ret = 0;

	DBGOUT("%s\n", __func__);

	if (!pdata) {
		dev_err(pdev->dev.parent, "No platform init data supplied\n");
		return -ENODEV;
	}

	power = kzalloc(sizeof(struct nxe1100_power), GFP_KERNEL);
	if (!power)
		return -ENOMEM;

	power->dev = &pdev->dev;
	power->iodev = iodev;

	/* get platdata */
	memcpy(&power->platdata, pdata, sizeof(struct nxe1100_pdata));
	pdata = &power->platdata;

	platform_set_drvdata(pdev, power);
	i2c = power->iodev->i2c;

	usb = &power->usb;
	battery = &power->battery;
	wall = &power->wall;

	snprintf(power->wall_name, sizeof(power->wall_name),
		 "nxe1100-wall");
	snprintf(power->battery_name, sizeof(power->wall_name),
		 "nxe1100-battery");
	snprintf(power->usb_name, sizeof(power->wall_name),
		 "nxe1100-usb");

	/* Setup "End of Charge" */
	/* If EOC value equals 0,
	 * remain value set from bootloader or default value */
	if (pdata->eoc_mA >= 10 && pdata->eoc_mA <= 45) {
		nxe1100_update_reg(i2c, NXE1100_REG_BATSET1,
				(pdata->eoc_mA / 5 - 2) << 5, 0x7 << 5);
	} else if (pdata->eoc_mA == 0) {
		dev_dbg(power->dev,
			"EOC value not set: leave it unchanged.\n");
	} else {
		dev_err(power->dev, "Invalid EOC value\n");
		ret = -EINVAL;
		goto err_kmalloc;
	}

	nxe1100_set_chg_current(i2c, NXE1100_PWR_SRC_BAT, pdata->chg_current);
	nxe1100_set_chg_current(i2c, NXE1100_PWR_SRC_ADP, pdata->adp_ilim_current);
	nxe1100_set_chg_current(i2c, NXE1100_PWR_SRC_USB, pdata->usb_ilim_current);

	/* Feul Gauge Enable */
	nxe1100_update_reg(i2c, NXE1100_REG_FG_CTRL, 0x1 << NXE1100_POS_FG_CTRL_FG_EN, 0x1 << NXE1100_POS_FG_CTRL_FG_EN);

	/* Setup Charge Restart Level */
	switch (pdata->restart) {
	case 100:
		nxe1100_update_reg(i2c, NXE1100_REG_BATSET1, 0x1 << 3, 0x3 << 3);
		break;
	case 150:
		nxe1100_update_reg(i2c, NXE1100_REG_BATSET1, 0x0 << 3, 0x3 << 3);
		break;
	case 200:
		nxe1100_update_reg(i2c, NXE1100_REG_BATSET1, 0x2 << 3, 0x3 << 3);
		break;
	case -1:
		nxe1100_update_reg(i2c, NXE1100_REG_BATSET1, 0x3 << 3, 0x3 << 3);
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
		nxe1100_update_reg(i2c, NXE1100_REG_TT_FULL_H, 0x0 << 4, 0x3 << 4);
		break;
	case 6:
		nxe1100_update_reg(i2c, NXE1100_REG_TT_FULL_H, 0x1 << 4, 0x3 << 4);
		break;
	case 7:
		nxe1100_update_reg(i2c, NXE1100_REG_TT_FULL_H, 0x2 << 4, 0x3 << 4);
		break;
	case -1:
		nxe1100_update_reg(i2c, NXE1100_REG_TT_FULL_H, 0x3 << 4, 0x3 << 4);
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

	wall->name = power->wall_name;
	wall->type = POWER_SUPPLY_TYPE_MAINS;
	wall->properties = nxe1100_wall_props;
	wall->num_properties = ARRAY_SIZE(nxe1100_wall_props);
	wall->get_property = nxe1100_wall_get_prop;
	ret = power_supply_register(&pdev->dev, wall);
	if (ret)
		goto err_kmalloc;

	usb->name = power->usb_name,
	usb->type = POWER_SUPPLY_TYPE_USB;
	usb->properties = nxe1100_usb_props;
	usb->num_properties = ARRAY_SIZE(nxe1100_usb_props);
	usb->get_property = nxe1100_usb_get_prop;
	ret = power_supply_register(&pdev->dev, usb);
	if (ret)
		goto err_wall;

	power->have_battery = 1;

	if (power->have_battery) {
		battery->name = power->battery_name;
		battery->type = POWER_SUPPLY_TYPE_BATTERY;
		battery->properties = nxe1100_bat_props;
		battery->num_properties = ARRAY_SIZE(nxe1100_bat_props);
		battery->get_property = nxe1100_bat_get_prop;
		battery->use_for_apm = 1;
		ret = power_supply_register(&pdev->dev, battery);
		if (ret)
			goto err_usb;
	}

#ifdef	NXE1100_USED_TIMER_THREAD
	/* Initilialize safety timer */
	init_timer(&power->rdstate_timer);
	power->rdstate_timer.function	= nxe1100_rdstate_timer_expired;
	power->rdstate_timer.data		= (unsigned long) power;
	power->rdstate_timer.expires	= jiffies + msecs_to_jiffies(1000);
	power->rdstate_repeat_time		= pdata->rdstate_periodic;

	add_timer(&power->rdstate_timer);
#else

#ifdef	NXE1100_RDSTATE_ONLY
	irq = iodev->irq_base + NXE1100_IRQ_CHGCTRL_RDSTATESHIFT;
#else
	irq = iodev->irq_base + NXE1100_IRQ_CHGCTRL_VADPDETS;
#endif
	ret = request_threaded_irq(irq, NULL, nxe1100_pwr_src_irq,
				   IRQF_TRIGGER_RISING, "Power source",
				   power);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request PWR SRC IRQ %d: %d\n",
			irq, ret);
		goto err_battery;
	}

#ifndef	NXE1100_RDSTATE_ONLY
	irq = iodev->irq_base + NXE1100_IRQ_CHGCTRL_VUSBDETS;
	ret = request_threaded_irq(irq, NULL, nxe1100_pwr_src_irq,
				   IRQF_TRIGGER_RISING, "Power source",
				   power);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request PWR SRC IRQ %d: %d\n",
			irq, ret);
		goto err_adp_irq;
	}
#endif
#endif	// #ifdef NXE1100_USED_TIMER_THREAD

	nxe1100_config_battery(iodev);

	return 0;
#ifndef	NXE1100_USED_TIMER_THREAD
err_adp_irq:
#ifdef	NXE1100_RDSTATE_ONLY
	irq = iodev->irq_base + NXE1100_IRQ_CHGCTRL_RDSTATESHIFT;
#else
	irq = iodev->irq_base + NXE1100_IRQ_CHGCTRL_VADPDETS;
#endif
	free_irq(irq, power);
err_battery:
#endif	// #ifdef NXE1100_USED_TIMER_THREAD
	if (power->have_battery)
		power_supply_unregister(battery);
err_usb:
	power_supply_unregister(usb);
err_wall:
	power_supply_unregister(wall);
err_kmalloc:
	kfree(power);
	return ret;
}

static int __devexit nxe1100_power_remove(struct platform_device *pdev)
{
	struct nxe1100_power *charger = platform_get_drvdata(pdev);

	DBGOUT("%s\n", __func__);

	power_supply_unregister(&charger->battery);
	kfree(charger);

	return 0;
}

static const struct platform_device_id nxe1100_power_id[] = {
	{ "nxe1100-battery", TYPE_NXE1100 },
	{ }
};

static struct platform_driver nxe1100_power_driver = {
	.driver = {
		.name = "nxe1100-power",
		.owner = THIS_MODULE,
	},
	.probe = nxe1100_power_probe,
	.remove = __devexit_p(nxe1100_power_remove),
	.id_table = nxe1100_power_id,
};

#if 1
module_platform_driver(nxe1100_power_driver);
#else
static int __init nxe1100_battery_init(void)
{
	return platform_driver_register(&nxe1100_power_driver);
}
subsys_initcall(nxe1100_battery_init);

static void __exit nxe1100_battery_cleanup(void)
{
	platform_driver_unregister(&nxe1100_power_driver);
}
module_exit(nxe1100_battery_cleanup);
#endif

MODULE_DESCRIPTION("NXE1100 battery control driver");
MODULE_AUTHOR("Bongkwan Kook <kook@nexell.co.kr>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nxe1100-battery");
