/*
 * nxe1100.c - Voltage regulator driver for the NXE1100
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
#include <nxe1100.h>
#include <nxe1100-private.h>

/*
 * Debug
 */
#if (0)
#define DBGOUT(msg...)		{ printk("nxe1100-regulator: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define	NX_PMIC_I2C_ID		"nxe1100"
#define	NX_PMIC_DRV_ID		"nxe1100-pmic"

struct nxe1100_data {
	struct device			*dev;
	struct nxe1100_dev		*iodev;
	int						num_regulators;
	struct regulator_dev	**rdev;
	int						ramp_delay;		/* in mV/us */

	u8                      buck_vol[3];	/* voltages for selection */
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
static const struct voltage_map_desc buck123_voltage_map_desc = {
	.min = 600000,	.max = 3500000,	.step = 12500,
};
static const struct voltage_map_desc ldo1245_voltage_map_desc = {
	.min = 900000,	.max = 3500000,	.step = 25000,
};
static const struct voltage_map_desc ldo3_voltage_map_desc = {
	.min = 600000,	.max = 3500000,	.step = 25000,
};

/* current map in mA */
static const struct voltage_map_desc charger_current_map_desc = {
	.min = 200,		.max = 950,		.step = 50,
};

static const struct voltage_map_desc topoff_current_map_desc = {
	.min = 50,		.max = 200,		.step = 10,
};

static const struct voltage_map_desc *ldo_voltage_map[] = {
	[NXE1100_LDO1] = &ldo1245_voltage_map_desc,			/* LDO1 */
	[NXE1100_LDO2] = &ldo1245_voltage_map_desc,			/* LDO2 */
	[NXE1100_LDO3] = &ldo3_voltage_map_desc,			/* LDO3 */
	[NXE1100_LDO4] = &ldo1245_voltage_map_desc,			/* LDO4 */
	[NXE1100_LDO5] = &ldo1245_voltage_map_desc,			/* LDO5 */
	[NXE1100_BUCK1] = &buck123_voltage_map_desc,		/* DCDC1 */
	[NXE1100_BUCK2] = &buck123_voltage_map_desc,		/* DCDC2 */
	[NXE1100_BUCK3] = &buck123_voltage_map_desc,		/* DCDC3 */
#if 0
	[NXE1100_CHARGER] = &charger_current_map_desc,
	[NXE1100_CHARGER_TOPOFF] = &topoff_current_map_desc,
#endif
};

static int nxe1100_list_voltage(struct regulator_dev *rdev,
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

static int nxe1100_get_enable_register(struct regulator_dev *rdev,
		int *reg, int *mask, int *pattern)
{
	int rid = rdev_get_id(rdev);

	DBGOUT("%s\n", __func__);

	switch (rid) {
	case NXE1100_BUCK1 ... NXE1100_BUCK3:
		*reg = NXE1100_REG_DC1CTL + ((rid - NXE1100_BUCK1) * 2);
		*mask = 0x01;
		*pattern = 0x01;
		break;
	case NXE1100_LDO1 ... NXE1100_LDO5:
		*reg = NXE1100_REG_LDOEN1;
		*mask = 1 << (rid - NXE1100_LDO1);
		*pattern = 1 << (rid - NXE1100_LDO1);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nxe1100_get_disable_register(struct regulator_dev *rdev,
	int *reg, int *mask, int *pattern)
{
	int rid = rdev_get_id(rdev);

	DBGOUT("%s\n", __func__);

	switch (rid) {
	case NXE1100_BUCK1 ... NXE1100_BUCK3:
		*reg = NXE1100_REG_DC1CTL + ((rid - NXE1100_BUCK1) * 2);
		*mask = 0x02;
		*pattern = 0x02;
		break;
	case NXE1100_LDO1 ... NXE1100_LDO5:
		*reg = NXE1100_REG_LDOEN1;
		*mask = 1 << (rid - NXE1100_LDO1);
		*pattern = 1 << (rid - NXE1100_LDO1);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nxe1100_reg_is_enabled(struct regulator_dev *rdev)
{
	struct nxe1100_data *nxe1100 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = nxe1100->iodev->i2c;
	int ret, reg, mask, pattern;
	u8 val;

	DBGOUT("%s\n", __func__);

	ret = nxe1100_get_enable_register(rdev, &reg, &mask, &pattern);
	if (ret == -EINVAL)
		return 1; /* "not controllable" */
	else if (ret)
		return ret;

	ret = nxe1100_read_reg(i2c, reg, &val);
	if (ret)
		return ret;

	return (val & mask) == pattern;
}

static int nxe1100_reg_enable(struct regulator_dev *rdev)
{
	struct nxe1100_data *nxe1100 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = nxe1100->iodev->i2c;
	int ret, reg, mask, pattern;

	DBGOUT("%s\n", __func__);

	ret = nxe1100_get_enable_register(rdev, &reg, &mask, &pattern);
	if (ret)
		return ret;

	return nxe1100_update_reg(i2c, reg, pattern, mask);
}

static int nxe1100_reg_disable(struct regulator_dev *rdev)
{
	struct nxe1100_data *nxe1100 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = nxe1100->iodev->i2c;
	int ret, reg, mask, pattern;

	DBGOUT("%s\n", __func__);

	ret = nxe1100_get_disable_register(rdev, &reg, &mask, &pattern);
	if (ret)
		return ret;

	return nxe1100_update_reg(i2c, reg, ~pattern, mask);
}

static int nxe1100_get_voltage_register(struct regulator_dev *rdev,
				int *_reg, int *_shift, int *_mask)
{
	int ldo = rdev_get_id(rdev);
	int reg, shift = 0, mask = 0xff;

	switch (ldo) {
	case NXE1100_BUCK1 ... NXE1100_BUCK3:
		reg = NXE1100_REG_DC1DAC + (ldo - NXE1100_BUCK1);
		mask = 0xff;
		break;
	case NXE1100_LDO1 ... NXE1100_LDO5:
		reg = NXE1100_REG_LDO1DAC + (ldo - NXE1100_LDO1);
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

static int nxe1100_get_voltage(struct regulator_dev *rdev)
{
	struct nxe1100_data *nxe1100 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = nxe1100->iodev->i2c;
	int reg, shift = 0, mask, ret;
	u8 val;

	DBGOUT("%s\n", __func__);

	ret = nxe1100_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret)
		return ret;

	ret = nxe1100_read_reg(i2c, reg, &val);
	if (ret)
		return ret;

	val >>= shift;
	val &= mask;

	return nxe1100_list_voltage(rdev, val);
}

static inline int nxe1100_get_voltage_proper_val(
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

static int nxe1100_set_voltage_ldobuck(struct regulator_dev *rdev,
				   int min_uV, int max_uV, unsigned *selector)
{
	struct nxe1100_data *nxe1100 = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = nxe1100->iodev->i2c;
	int min_vol = min_uV, max_vol = max_uV;
	const struct voltage_map_desc *desc;
	int rid = rdev_get_id(rdev);
	int reg, shift = 0, mask, ret;
	int i = 0;
	u8	org;

	DBGOUT("%s\n", __func__);

	if (rid >= ARRAY_SIZE(ldo_voltage_map))
		return -EINVAL;

	desc = ldo_voltage_map[rid];

	i = nxe1100_get_voltage_proper_val(desc, min_vol, max_vol);
	if (i < 0)
		return i;

	ret = nxe1100_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret)
		return ret;

	nxe1100_read_reg(i2c, reg, &org);
	org = (org & mask) >> shift;

	ret = nxe1100_update_reg(i2c, reg, i << shift, mask << shift);
	*selector = i;

	if (rid == NXE1100_BUCK1 || rid == NXE1100_BUCK2 ||
			rid == NXE1100_BUCK3) {
		/* If the voltage is increasing */
		if (org < i)
			udelay(DIV_ROUND_UP(desc->step * (i - org),
						nxe1100->ramp_delay));
	}

	return ret;
}

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


static struct regulator_ops nxe1100_ldo_ops = {
	.list_voltage			= nxe1100_list_voltage,
	.is_enabled				= nxe1100_reg_is_enabled,
	.enable					= nxe1100_reg_enable,
	.disable				= nxe1100_reg_disable,
	.get_voltage			= nxe1100_get_voltage,
	.set_voltage			= nxe1100_set_voltage_ldobuck,
	.set_suspend_enable		= nxe1100_reg_enable,
	.set_suspend_disable	= nxe1100_reg_disable,
};

static struct regulator_ops nxe1100_buck_ops = {
	.list_voltage			= nxe1100_list_voltage,
	.is_enabled				= nxe1100_reg_is_enabled,
	.enable					= nxe1100_reg_enable,
	.disable				= nxe1100_reg_disable,
	.get_voltage			= nxe1100_get_voltage,
	.set_voltage			= nxe1100_set_voltage_ldobuck,
	.set_suspend_enable		= nxe1100_reg_enable,
	.set_suspend_disable	= nxe1100_reg_disable,
};

#if 0
static struct regulator_ops nxe1100_others_ops = {
	.is_enabled				= nxe1100_reg_is_enabled,
	.enable					= nxe1100_reg_enable,
	.disable				= nxe1100_reg_disable,
	.set_suspend_enable		= nxe1100_reg_enable,
	.set_suspend_disable	= nxe1100_reg_disable,
};
#endif

#if 0
static int nxe1100_set_voltage_ldobuck_wrap(struct regulator_dev *rdev,
		int min_uV, int max_uV)
{
	unsigned dummy;

	return nxe1100_set_voltage_ldobuck(rdev, min_uV, max_uV, &dummy);
}

static struct regulator_ops nxe1100_charger_ops = {
	.is_enabled			= nxe1100_reg_is_enabled,
	.enable				= nxe1100_reg_enable,
	.disable			= nxe1100_reg_disable,
	.get_current_limit	= nxe1100_get_voltage,
	.set_current_limit	= nxe1100_set_voltage_ldobuck_wrap,
};

static struct regulator_ops nxe1100_charger_fixedstate_ops = {
	.is_enabled			= nxe1100_reg_is_enabled,
	.get_current_limit	= nxe1100_get_voltage,
	.set_current_limit	= nxe1100_set_voltage_ldobuck_wrap,
};
#endif

#define regulator_desc_ldo(num)		{	\
	.name		= "LDO"#num,			\
	.id			= NXE1100_LDO##num,		\
	.ops		= &nxe1100_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,	\
	.owner		= THIS_MODULE,			\
}
#define regulator_desc_buck(num)	{	\
	.name		= "BUCK"#num,			\
	.id			= NXE1100_BUCK##num,	\
	.ops		= &nxe1100_buck_ops,	\
	.type		= REGULATOR_VOLTAGE,	\
	.owner		= THIS_MODULE,			\
}

static struct regulator_desc regulators[] = {
	regulator_desc_ldo(1),
	regulator_desc_ldo(2),
	regulator_desc_ldo(3),
	regulator_desc_ldo(4),
	regulator_desc_ldo(5),
	regulator_desc_buck(1),
	regulator_desc_buck(2),
	regulator_desc_buck(3),
#if 0
	{
		.name	= "CHARGER",
		.id		= NXE1100_CHARGER,
		.ops	= &nxe1100_charger_ops,
		.type	= REGULATOR_CURRENT,
		.owner	= THIS_MODULE,
	}, {
		.name	= "CHARGER_TOPOFF",
		.id		= NXE1100_CHARGER_TOPOFF,
		.ops	= &nxe1100_charger_fixedstate_ops,
		.type	= REGULATOR_CURRENT,
		.owner	= THIS_MODULE,
	},
#endif
};

static __devinit int nxe1100_pmic_probe(struct platform_device *pdev)
{
	struct nxe1100_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct nxe1100_pdata *pdata = dev_get_platdata(iodev->dev);
	struct regulator_dev **rdev;
	struct nxe1100_data *nxe1100;
	struct i2c_client *i2c;
	int i, ret, size;

	DBGOUT("%s\n", __func__);

	if (!pdata) {
		dev_err(pdev->dev.parent, "No platform init data supplied\n");
		return -ENODEV;
	}

	nxe1100 = kzalloc(sizeof(struct nxe1100_data), GFP_KERNEL);
	if (!nxe1100)
		return -ENOMEM;

	size = sizeof(struct regulator_dev *) * pdata->num_regulators;
	nxe1100->rdev = kzalloc(size, GFP_KERNEL);
	if (!nxe1100->rdev) {
		kfree(nxe1100);
		return -ENOMEM;
	}

	rdev = nxe1100->rdev;
	nxe1100->dev = &pdev->dev;
	nxe1100->iodev = iodev;
	nxe1100->num_regulators = pdata->num_regulators;
	platform_set_drvdata(pdev, nxe1100);
	i2c = nxe1100->iodev->i2c;

	/* NOTE: */
	/* For unused GPIO NOT marked as -1 (thereof equal to 0)  WARN_ON */
	/* will be displayed */

	/* Check if NXE1100 voltage selection GPIOs are defined */
	for (i = 0; i < 3; i++) {
		nxe1100->buck_vol[i] = ret =
			nxe1100_get_voltage_proper_val(
					&buck123_voltage_map_desc,
					pdata->buck_voltage[i],
					pdata->buck_voltage[i] +
					buck123_voltage_map_desc.step);
		if (ret < 0)
			goto err_free_mem;
	}

#if 0
	for (i = 0; i < 3; i++) {
		ret = nxe1100_write_reg(i2c, NXE1100_REG_DC1DAC + i, nxe1100->buck_vol[i]);
		if (ret < 0)
		{
			goto err_free_mem;
		}
	}
#endif

	/* Misc Settings */
	nxe1100->ramp_delay = 14; /* set 14mV/us, which is the default */

	for (i = 0; i < pdata->num_regulators; i++) {
		const struct voltage_map_desc *desc;
		int id = pdata->regulators[i].id;
		int index = id - NXE1100_LDO2;

		desc = ldo_voltage_map[id];
		if (desc)
			regulators[index].n_voltages =
				(desc->max - desc->min) / desc->step + 1;

		rdev[i] = regulator_register(&regulators[id], nxe1100->dev,
				pdata->regulators[i].initdata, nxe1100, NULL);
		if (IS_ERR(rdev[i])) {
			ret = PTR_ERR(rdev[i]);
			dev_err(nxe1100->dev, "regulator init failed\n");
			rdev[i] = NULL;
			goto err;
		}
	}


	return 0;
err:
	for (i = 0; i < nxe1100->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

err_free_mem:
	kfree(nxe1100->rdev);
	kfree(nxe1100);

	return ret;
}

static int __devexit nxe1100_pmic_remove(struct platform_device *pdev)
{
	struct nxe1100_data *nxe1100 = platform_get_drvdata(pdev);
	struct regulator_dev **rdev = nxe1100->rdev;
	int i;

	DBGOUT("%s\n", __func__);

	for (i = 0; i < nxe1100->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

	kfree(nxe1100->rdev);
	kfree(nxe1100);

	return 0;
}

static const struct platform_device_id nxe1100_pmic_id[] = {
	{ "nxe1100-pmic", TYPE_NXE1100 },
	{ }
};
MODULE_DEVICE_TABLE(platform, nxe1100_pmic_id);

static struct platform_driver nxe1100_pmic_driver = {
	.driver = {
		.name = "nxe1100-pmic",
		.owner = THIS_MODULE,
	},
	.probe = nxe1100_pmic_probe,
	.remove = __devexit_p(nxe1100_pmic_remove),
	.id_table = nxe1100_pmic_id,
};

static int __init nxe1100_pmic_init(void)
{
	int ret;

	DBGOUT("%s\n", __func__);

	ret = platform_driver_register(&nxe1100_pmic_driver);
	if (ret != 0)
		printk("Failed to register nxe1100 pmic driver: %d\n", ret);

	return 0;
}
subsys_initcall(nxe1100_pmic_init);

static void __exit nxe1100_pmic_cleanup(void)
{
	DBGOUT("%s\n", __func__);

	platform_driver_unregister(&nxe1100_pmic_driver);
}
module_exit(nxe1100_pmic_cleanup);

MODULE_DESCRIPTION("NXE1100 voltage regulator driver");
MODULE_AUTHOR("Bongkwan Kook <kook@nexell.co.kr>");
MODULE_LICENSE("GPL");

