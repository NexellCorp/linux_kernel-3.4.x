/*
 * nxe2000-regulator.c - Voltage regulator driver for the NXE2000
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
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>

/* nexell soc headers */
#include <nxe1999.h>
#include <nxe2000-private.h>

/*
 * Debug
 */
#if (0)
#define DBGOUT(msg...)		{ printk("nxe2000-regulator: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define	NX_PMIC_I2C_ID		"nxe2000"
#define	NX_PMIC_DRV_ID		"nxe2000-pmic"

#define CFG_NXE2000_SUSPEND_POWER_OFF   (0)


struct nxe2000_regul {
	struct device				*dev;
	struct nxe2000_dev			*iodev;
	int							num_regulators;
	struct regulator_dev		**rdev;

#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
	struct delayed_work			suspend_on_work;
	struct workqueue_struct		*suspend_on_wqueue;
#endif

	int						ramp_delay;		/* in mV/us */

	uint8_t                 slp_prio_buck[NXE2000_NUM_BUCK];   // 0 ~ 14, off => 15
	uint8_t                 slp_prio_ldo[NXE2000_NUM_LDO];
	uint8_t                 slp_prio_pso[NXE2000_NUM_PSO];

	uint8_t                 slp_buck_vol[NXE2000_NUM_BUCK];	/* voltages for selection */
	uint8_t                 slp_ldo_vol[NXE2000_NUM_LDO];	/* voltages for selection */
};

struct voltage_map_desc {
	int min;
	int max;
	int step;
};

/* Voltage maps */
/*
 .min = 6000000		// Uint = 1uV
 .step = 12500		// Uint = 1uV
 .max = 35000000		// Uint = 1uV
*/
static const struct voltage_map_desc buck_x_voltage_map_desc = {
	.min = 600000,	.max = 3500000,	.step = 12500,
};
static const struct voltage_map_desc ldo_x_voltage_map_desc = {
	.min = 900000,	.max = 3500000,	.step = 25000,
};
static const struct voltage_map_desc ldo_56_voltage_map_desc = {
	.min = 600000,	.max = 3500000,	.step = 25000,
};
static const struct voltage_map_desc ldortc_1_voltage_map_desc = {
	.min = 1700000,	.max = 3500000,	.step = 25000,
};
static const struct voltage_map_desc ldortc_2_voltage_map_desc = {
	.min = 900000,	.max = 3500000,	.step = 25000,
};

/* current map in mA */
static const struct voltage_map_desc charger_current_map_desc = {
	.min = 200,		.max = 950,		.step = 50,
};

static const struct voltage_map_desc topoff_current_map_desc = {
	.min = 50,		.max = 200,		.step = 10,
};

static const struct voltage_map_desc *ldo_voltage_map[] = {
	[NXE2000_LDO1]      = &ldo_x_voltage_map_desc,      /* LDO1 */
	[NXE2000_LDO2]      = &ldo_x_voltage_map_desc,      /* LDO2 */
	[NXE2000_LDO3]      = &ldo_x_voltage_map_desc,      /* LDO3 */
	[NXE2000_LDO4]      = &ldo_x_voltage_map_desc,      /* LDO4 */
	[NXE2000_LDO5]      = &ldo_56_voltage_map_desc,     /* LDO5 */
	[NXE2000_LDO6]      = &ldo_56_voltage_map_desc,     /* LDO6 */
	[NXE2000_LDO7]      = &ldo_x_voltage_map_desc,      /* LDO7 */
	[NXE2000_LDO8]      = &ldo_x_voltage_map_desc,      /* LDO8 */
	[NXE2000_LDO9]      = &ldo_x_voltage_map_desc,      /* LDO9 */
	[NXE2000_LDO10]     = &ldo_x_voltage_map_desc,      /* LDO10 */
	[NXE2000_LDORTC1]   = &ldortc_1_voltage_map_desc,   /* LDORTC1 */
	[NXE2000_LDORTC2]   = &ldortc_2_voltage_map_desc,   /* LDORTC2 */
	[NXE2000_BUCK1]     = &buck_x_voltage_map_desc,     /* DCDC1 */
	[NXE2000_BUCK2]     = &buck_x_voltage_map_desc,     /* DCDC2 */
	[NXE2000_BUCK3]     = &buck_x_voltage_map_desc,     /* DCDC3 */
	[NXE2000_BUCK4]     = &buck_x_voltage_map_desc,     /* DCDC4 */
	[NXE2000_BUCK5]     = &buck_x_voltage_map_desc,     /* DCDC5 */
#if 0
	[NXE2000_CHARGER] = &charger_current_map_desc,
	[NXE2000_CHARGER_TOPOFF] = &topoff_current_map_desc,
#endif
};

static unsigned int nxe2000_regul_suspend_status;

static int nxe2000_list_voltage(struct regulator_dev *rdev,
		unsigned int selector)
{
	const struct voltage_map_desc *desc;
	int rid = rdev_get_id(rdev);
	int val;

	if (rid >= ARRAY_SIZE(ldo_voltage_map) ||
			rid < 0)
		return -EINVAL;

	desc = ldo_voltage_map[rid];
	if (desc == NULL)
		return -EINVAL;

	val = desc->min + desc->step * selector;
	if (val > desc->max)
		return -EINVAL;

	return val;
}

static int nxe2000_get_enable_reg(
		int rid, int *reg, int *mask, int *pattern)
{
	DBGOUT("%s\n", __func__);

	switch (rid) {
	case NXE2000_BUCK1 ... NXE2000_BUCK5:
		*reg = NXE2000_REG_DC1CTL + ((rid - NXE2000_BUCK1) * 2);
		*mask = 0x01;
		*pattern = 0x01;
		break;
	case NXE2000_LDO1 ... NXE2000_LDO8:
		*reg = NXE2000_REG_LDOEN1;
		*mask = 1 << (rid - NXE2000_LDO1);
		*pattern = 1 << (rid - NXE2000_LDO1);
		break;
	case NXE2000_LDO9 ... NXE2000_LDO10:
		*reg = NXE2000_REG_LDOEN2;
		*mask = 1 << (rid - NXE2000_LDO9);
		*pattern = 1 << (rid - NXE2000_LDO9);
		break;
	case NXE2000_LDORTC1 ... NXE2000_LDORTC2:
		*reg = NXE2000_REG_LDOEN2;
		*mask = 1 << ((rid - NXE2000_LDORTC1) + 4);
		*pattern = 1 << ((rid - NXE2000_LDORTC1) + 4);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nxe2000_get_disable_reg(struct regulator_dev *rdev,
	int *reg, int *mask, int *pattern)
{
	int rid = rdev_get_id(rdev);

	DBGOUT("%s\n", __func__);

	switch (rid) {
	case NXE2000_BUCK1 ... NXE2000_BUCK5:
		*reg = NXE2000_REG_DC1CTL + ((rid - NXE2000_BUCK1) * 2);
		*mask = 0x02;
		*pattern = 0x02;
		break;
	case NXE2000_LDO1 ... NXE2000_LDO8:
		*reg = NXE2000_REG_LDOEN1;
		*mask = 1 << (rid - NXE2000_LDO1);
		*pattern = 1 << (rid - NXE2000_LDO1);
		break;
	case NXE2000_LDO9 ... NXE2000_LDO10:
		*reg = NXE2000_REG_LDOEN2;
		*mask = 1 << (rid - NXE2000_LDO9);
		*pattern = 1 << (rid - NXE2000_LDO9);
		break;
	case NXE2000_LDORTC1 ... NXE2000_LDORTC2:
		*reg = NXE2000_REG_LDOEN2;
		*mask = 1 << ((rid - NXE2000_LDORTC1) + 4);
		*pattern = 1 << ((rid - NXE2000_LDORTC1) + 4);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nxe2000_reg_is_enabled(struct regulator_dev *rdev)
{
	struct nxe2000_regul *regul = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = regul->iodev->i2c;
	int rid = rdev_get_id(rdev);
	int ret, reg, mask, pattern;
	u8 val;

	DBGOUT("%s\n", __func__);

	ret = nxe2000_get_enable_reg(rid, &reg, &mask, &pattern);
	if (ret == -EINVAL)
		return 1; /* "not controllable" */
	else if (ret)
		return ret;

	ret = nxe2000_read(i2c, reg, &val);
	if (ret)
		return ret;

	return (val & mask) == pattern;
}

static int nxe2000_reg_enable(struct regulator_dev *rdev)
{
	struct nxe2000_regul *regul = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = regul->iodev->i2c;
	int rid = rdev_get_id(rdev);
	int ret, reg, mask, pattern;

	DBGOUT("%s\n", __func__);

	ret = nxe2000_get_enable_reg(rid, &reg, &mask, &pattern);
	if (ret)
		return ret;

	return nxe2000_update(i2c, reg, pattern, mask);
}

static int nxe2000_reg_disable(struct regulator_dev *rdev)
{
	struct nxe2000_regul *regul = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = regul->iodev->i2c;
	int ret, reg, mask, pattern;

	DBGOUT("%s\n", __func__);

	ret = nxe2000_get_disable_reg(rdev, &reg, &mask, &pattern);
	if (ret)
		return ret;

	return nxe2000_update(i2c, reg, ~pattern, mask);
}

static int nxe2000_get_voltage_register(struct regulator_dev *rdev,
				int *_reg, int *_shift, int *_mask)
{
	int ldo = rdev_get_id(rdev);
	int reg, shift = 0, mask = 0xff;

	switch (ldo) {
	case NXE2000_BUCK1 ... NXE2000_BUCK5:
		reg = NXE2000_REG_DC1VOL + (ldo - NXE2000_BUCK1);
		mask = 0xff;
		break;
	case NXE2000_LDO1 ... NXE2000_LDORTC2:
		reg = NXE2000_REG_LDO1VOL + (ldo - NXE2000_LDO1);
		mask = 0x7f;
		break;
	default:
		return -EINVAL;
	}

	*_reg = reg;
	*_shift = shift;
	*_mask = mask;

	return 0;
}

static int nxe2000_get_voltage(struct regulator_dev *rdev)
{
	struct nxe2000_regul *regul = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = regul->iodev->i2c;
	int reg, shift = 0, mask, ret;
	u8 val;

	DBGOUT("%s\n", __func__);

	ret = nxe2000_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret)
		return ret;

	ret = nxe2000_read(i2c, reg, &val);
	if (ret)
		return ret;

	val >>= shift;
	val &= mask;

	return nxe2000_list_voltage(rdev, val);
}

static inline int nxe2000_get_voltage_proper_val(
		const struct voltage_map_desc *desc,
		int min_vol, int max_vol)
{
	int i = 0;

	if (desc == NULL)
		return -EINVAL;

	if (max_vol < desc->min || min_vol > desc->max)
		return -EINVAL;

	while (desc->min + desc->step * i < min_vol &&
			desc->min + desc->step * i < desc->max)
		i++;

	if (desc->min + desc->step * i > max_vol)
		return -EINVAL;

	return i;
}

static int nxe2000_set_voltage_ldobuck(struct regulator_dev *rdev,
				   int min_uV, int max_uV, unsigned *selector)
{
	struct nxe2000_regul *regul = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = regul->iodev->i2c;
	int min_vol = min_uV, max_vol = max_uV;
	const struct voltage_map_desc *desc;
	int rid = rdev_get_id(rdev);
	int reg, shift = 0, mask, ret;
	int i = 0;
	u8	org;

	DBGOUT("%s\n", __func__);

	if (nxe2000_regul_suspend_status)
		return -EBUSY;

	if (rid >= ARRAY_SIZE(ldo_voltage_map))
		return -EINVAL;

	desc = ldo_voltage_map[rid];

	i = nxe2000_get_voltage_proper_val(desc, min_vol, max_vol);
	if (i < 0)
		return i;

	ret = nxe2000_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret)
		return ret;

	nxe2000_read(i2c, reg, &org);
	org = (org & mask) >> shift;

	ret = nxe2000_update(i2c, reg, i << shift, mask << shift);
	*selector = i;

	if( (rid >= NXE2000_BUCK1) && (rid <= NXE2000_BUCK5) )
	{
		/* If the voltage is increasing */
		if (org < i)
			udelay(DIV_ROUND_UP(desc->step * (i - org),
						regul->ramp_delay) * 5);
	}

	return ret;
}

#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
static void nxe2000_resume_regul_work(struct work_struct *work)
{
	struct nxe2000_regul	*regul = container_of(work,
		struct nxe2000_regul, suspend_on_work.work);
	struct i2c_client *i2c  = regul->iodev->i2c;
    int i, ret, reg, mask, pattern;

	dev_dbg(regul->dev, "Resume Power On\n");

	for (i = 0; i < NXE2000_NUM_LDO; i++)
	{
		if (regul->slp_prio_ldo[i] == 0xF)
		{
			ret = nxe2000_get_enable_reg((NXE2000_LDO1 + i), &reg, &mask, &pattern);
			if (ret)
				continue;

			nxe2000_update(i2c, reg, pattern, mask);
		}
	}
}
#endif

static inline void buck1_gpio_set(int gpio1, int gpio2, int v)
{
	DBGOUT("%s\n", __func__);

	gpio_set_value(gpio1, v & 0x1);
	gpio_set_value(gpio2, (v >> 1) & 0x1);
}

static inline void buck2_gpio_set(int gpio, int v)
{
	DBGOUT("%s\n", __func__);

	gpio_set_value(gpio, v & 0x1);
}


static struct regulator_ops nxe2000_ldo_ops = {
	.list_voltage			= nxe2000_list_voltage,
	.is_enabled				= nxe2000_reg_is_enabled,
	.enable					= nxe2000_reg_enable,
	.disable				= nxe2000_reg_disable,
	.get_voltage			= nxe2000_get_voltage,
	.set_voltage			= nxe2000_set_voltage_ldobuck,
	.set_suspend_enable		= nxe2000_reg_enable,
	.set_suspend_disable	= nxe2000_reg_disable,
};

static struct regulator_ops nxe2000_buck_ops = {
	.list_voltage			= nxe2000_list_voltage,
	.is_enabled				= nxe2000_reg_is_enabled,
	.enable					= nxe2000_reg_enable,
	.disable				= nxe2000_reg_disable,
	.get_voltage			= nxe2000_get_voltage,
	.set_voltage			= nxe2000_set_voltage_ldobuck,
	.set_suspend_enable		= nxe2000_reg_enable,
	.set_suspend_disable	= nxe2000_reg_disable,
};

#if 0
static struct regulator_ops nxe2000_others_ops = {
	.is_enabled				= nxe2000_reg_is_enabled,
	.enable					= nxe2000_reg_enable,
	.disable				= nxe2000_reg_disable,
	.set_suspend_enable		= nxe2000_reg_enable,
	.set_suspend_disable	= nxe2000_reg_disable,
};
#endif

#if 0
static int nxe2000_set_voltage_ldobuck_wrap(struct regulator_dev *rdev,
		int min_uV, int max_uV)
{
	unsigned dummy;

	return nxe2000_set_voltage_ldobuck(rdev, min_uV, max_uV, &dummy);
}

static struct regulator_ops nxe2000_charger_ops = {
	.is_enabled			= nxe2000_reg_is_enabled,
	.enable				= nxe2000_reg_enable,
	.disable			= nxe2000_reg_disable,
	.get_current_limit	= nxe2000_get_voltage,
	.set_current_limit	= nxe2000_set_voltage_ldobuck_wrap,
};

static struct regulator_ops nxe2000_charger_fixedstate_ops = {
	.is_enabled			= nxe2000_reg_is_enabled,
	.get_current_limit	= nxe2000_get_voltage,
	.set_current_limit	= nxe2000_set_voltage_ldobuck_wrap,
};
#endif

#define regulator_desc_ldo(num)		{		\
	.name			= "LDO"#num,			\
	.id				= NXE2000_LDO##num,		\
	.ops			= &nxe2000_ldo_ops,		\
	.type			= REGULATOR_VOLTAGE,	\
	.owner			= THIS_MODULE,			\
}
#define regulator_desc_ldortc(num)	{		\
	.name			= "LDORTC"#num,			\
	.id				= NXE2000_LDORTC##num,	\
	.ops			= &nxe2000_ldo_ops,		\
	.type			= REGULATOR_VOLTAGE,	\
	.owner			= THIS_MODULE,			\
}
#define regulator_desc_buck(num)	{		\
	.name			= "BUCK"#num,			\
	.id				= NXE2000_BUCK##num,	\
	.ops			= &nxe2000_buck_ops,	\
	.type			= REGULATOR_VOLTAGE,	\
	.owner			= THIS_MODULE,			\
}

static struct regulator_desc regulators[] = {
	regulator_desc_ldo(1),
	regulator_desc_ldo(2),
	regulator_desc_ldo(3),
	regulator_desc_ldo(4),
	regulator_desc_ldo(5),
	regulator_desc_ldo(6),
	regulator_desc_ldo(7),
	regulator_desc_ldo(8),
	regulator_desc_ldo(9),
	regulator_desc_ldo(10),
	regulator_desc_ldortc(1),
	regulator_desc_ldortc(2),
	regulator_desc_buck(1),
	regulator_desc_buck(2),
	regulator_desc_buck(3),
	regulator_desc_buck(4),
	regulator_desc_buck(5),
#if 0
	{
		.name	= "CHARGER",
		.id		= NXE2000_CHARGER,
		.ops	= &nxe2000_charger_ops,
		.type	= REGULATOR_CURRENT,
		.owner	= THIS_MODULE,
	}, {
		.name	= "CHARGER_TOPOFF",
		.id		= NXE2000_CHARGER_TOPOFF,
		.ops	= &nxe2000_charger_fixedstate_ops,
		.type	= REGULATOR_CURRENT,
		.owner	= THIS_MODULE,
	},
#endif
};

static __devinit int nxe2000_pmic_probe(struct platform_device *pdev)
{
	struct nxe2000_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct nxe2000_pdata *pdata = dev_get_platdata(iodev->dev);
	struct regulator_dev **rdev;
	struct nxe2000_regul *regul;
	struct i2c_client *i2c;
	int i, ret, size;

	DBGOUT("%s\n", __func__);
printk("PMU: %s\n", __func__);

	if (!pdata) {
		dev_err(pdev->dev.parent, "No platform init data supplied\n");
		return -ENODEV;
	}

	regul = kzalloc(sizeof(struct nxe2000_regul), GFP_KERNEL);
	if (!regul)
		return -ENOMEM;

	size = sizeof(struct regulator_dev *) * pdata->num_regulators;
	regul->rdev = kzalloc(size, GFP_KERNEL);
	if (!regul->rdev) {
		kfree(regul);
		return -ENOMEM;
	}

	rdev = regul->rdev;
	regul->dev = &pdev->dev;
	regul->iodev = iodev;
	regul->num_regulators = pdata->num_regulators;
	platform_set_drvdata(pdev, regul);
	i2c = regul->iodev->i2c;

	nxe2000_regul_suspend_status = 0;

#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
	regul->suspend_on_wqueue
		= create_singlethread_workqueue("nxe2000_suspend_work");
	INIT_DELAYED_WORK_DEFERRABLE(&regul->suspend_on_work,
						nxe2000_resume_regul_work);

	queue_delayed_work(regul->suspend_on_wqueue,
						&regul->suspend_on_work,
						HZ);
#endif

	/* NOTE: */
	/* For unused GPIO NOT marked as -1 (thereof equal to 0)  WARN_ON */
	/* will be displayed */

	/* Get to NXE2000 BUCK sleep voltage */
	for (i = 0; i < NXE2000_NUM_BUCK; i++) {
		regul->slp_buck_vol[i] = ret =
			nxe2000_get_voltage_proper_val(
					&buck_x_voltage_map_desc,
					pdata->slp_buck_vol[i],
					pdata->slp_buck_vol[i] +
					buck_x_voltage_map_desc.step);
		if (ret < 0)
			goto err_free_mem;
	}

	/* Set to NXE2000 BUCK sleep voltage */
	for (i = 0; i < NXE2000_NUM_BUCK; i++) {
		ret = nxe2000_write(i2c, NXE2000_REG_DC1VOL_SLP + i, regul->slp_buck_vol[i]);
		if (ret < 0)
		{
			goto err_free_mem;
		}
	}

	/* Get to NXE2000 LDO sleep voltage */
	for (i = 0; i < NXE2000_NUM_LDO; i++) {
		regul->slp_ldo_vol[i] = ret =
			nxe2000_get_voltage_proper_val(
					&buck_x_voltage_map_desc,
					pdata->slp_ldo_vol[i],
					pdata->slp_ldo_vol[i] +
					buck_x_voltage_map_desc.step);
		if (ret < 0)
			goto err_free_mem;
	}

	/* Set to NXE2000 LDO sleep voltage */
	for (i = 0; i < NXE2000_NUM_LDO; i++) {
		ret = nxe2000_write(i2c, NXE2000_REG_LDO1VOL_SLP + i, regul->slp_ldo_vol[i]);
		if (ret < 0)
		{
			goto err_free_mem;
		}
	}

    /* Set to DCDC SLP_SLOT */
	for (i = 0; i < NXE2000_NUM_BUCK; i++) {
        regul->slp_prio_buck[i] = pdata->slp_prio_buck[i];
		ret = nxe2000_update(i2c, NXE2000_REG_DC1SLP_SLOT + i, regul->slp_prio_buck[i], 0xF);
		if (ret < 0)
		{
			goto err_free_mem;
		}
	}

    /* Set to LDO SLP_SLOT */
	for (i = 0; i < NXE2000_NUM_LDO; i++) {
        regul->slp_prio_ldo[i] = pdata->slp_prio_ldo[i];
		ret = nxe2000_update(i2c, NXE2000_REG_LDO1SLP_SLOT + i, regul->slp_prio_ldo[i], 0xF);
		if (ret < 0)
		{
			goto err_free_mem;
		}
	}

    /* Set to PSO SLP_SLOT */
	for (i = 0; i < NXE2000_NUM_PSO; i++) {
        regul->slp_prio_pso[i] = pdata->slp_prio_pso[i];
		ret = nxe2000_update(i2c, NXE2000_REG_PSO0SLP_SLOT + i, regul->slp_prio_pso[i], 0xF);
		if (ret < 0)
		{
			goto err_free_mem;
		}
	}

	/* Misc Settings */
	regul->ramp_delay = 14; /* set 14mV/us, which is the default */

	for (i = 0; i < pdata->num_regulators; i++) {
		const struct voltage_map_desc *desc;
		int id = pdata->regulators[i].id;
		int index = id - NXE2000_LDO1;

		desc = ldo_voltage_map[id];
		if (desc)
			regulators[index].n_voltages =
				(desc->max - desc->min) / desc->step + 1;

		rdev[i] = regulator_register(&regulators[id], regul->dev,
				pdata->regulators[i].initdata, regul, NULL);
		if (IS_ERR(rdev[i])) {
			ret = PTR_ERR(rdev[i]);
			dev_err(regul->dev, "regulator init failed\n");
			rdev[i] = NULL;
			goto err;
		}
	}


	return 0;
err:
	for (i = 0; i < regul->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

err_free_mem:
	kfree(regul->rdev);
	kfree(regul);

	return ret;
}

static int __devexit nxe2000_pmic_remove(struct platform_device *pdev)
{
	struct nxe2000_regul *regul = platform_get_drvdata(pdev);
	struct regulator_dev **rdev = regul->rdev;
	int i;

	DBGOUT("%s\n", __func__);

	for (i = 0; i < regul->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

	kfree(regul->rdev);
	kfree(regul);

	return 0;
}

#ifdef CONFIG_PM
static int nxe2000_regulator_suspend(struct device *dev)
{
#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
	struct nxe2000_regul *regul = dev_get_drvdata(dev);
	struct i2c_client    *i2c   = regul->iodev->i2c;
    int i, ret, reg, mask, pattern;

	printk(KERN_INFO "PMU: %s\n", __func__);
	nxe2000_regul_suspend_status = 1;

	flush_delayed_work(&regul->suspend_on_work);

	for (i = 0; i < NXE2000_NUM_LDO; i++)
	{
		if (regul->slp_prio_ldo[i] == 0xF)
		{
			ret = nxe2000_get_enable_reg((NXE2000_LDO1 + i), &reg, &mask, &pattern);
			if (ret)
				continue;

			nxe2000_update(i2c, reg, ~pattern, mask);
		}
	}
#else

	printk(KERN_INFO "PMU: %s\n", __func__);
	nxe2000_regul_suspend_status = 1;
#endif
	return 0;
}

static int nxe2000_regulator_resume(struct device *dev)
{
#if (CFG_NXE2000_SUSPEND_POWER_OFF == 1)
	struct nxe2000_regul *regul = dev_get_drvdata(dev);

	printk(KERN_INFO "PMU: %s\n", __func__);

	if (regul->slp_prio_ldo[7] == 0xF)
	{
		queue_delayed_work(regul->suspend_on_wqueue,
					&regul->suspend_on_work,
					jiffies + msecs_to_jiffies(100));
	}
#endif

	printk(KERN_INFO "PMU: %s\n", __func__);
	nxe2000_regul_suspend_status = 0;

	return 0;
}

static const struct dev_pm_ops nxe2000_regulator_pm_ops = {
	.suspend	= nxe2000_regulator_suspend,
	.resume		= nxe2000_regulator_resume,
};
#endif


static const struct platform_device_id nxe2000_pmic_id[] = {
	{ "nxe2000-pmic", TYPE_NXE2000 },
	{ }
};
MODULE_DEVICE_TABLE(platform, nxe2000_pmic_id);

static struct platform_driver nxe2000_pmic_driver = {
	.driver = {
		.name = "nxe2000-pmic",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &nxe2000_regulator_pm_ops,
#endif
	},
	.probe = nxe2000_pmic_probe,
	.remove = __devexit_p(nxe2000_pmic_remove),
	.id_table = nxe2000_pmic_id,
};

static int __init nxe2000_pmic_init(void)
{
	int ret;

	DBGOUT("%s\n", __func__);

	ret = platform_driver_register(&nxe2000_pmic_driver);
	if (ret != 0)
		printk("Failed to register nxe2000 pmic driver: %d\n", ret);

	return 0;
}
subsys_initcall(nxe2000_pmic_init);

static void __exit nxe2000_pmic_cleanup(void)
{
	DBGOUT("%s\n", __func__);

	platform_driver_unregister(&nxe2000_pmic_driver);
}
module_exit(nxe2000_pmic_cleanup);

MODULE_DESCRIPTION("NXE2000 voltage regulator driver");
MODULE_AUTHOR("Bongkwan Kook <kook@nexell.co.kr>");
MODULE_LICENSE("GPL");

