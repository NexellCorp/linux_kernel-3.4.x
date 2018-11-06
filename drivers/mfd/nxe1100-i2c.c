/*
 * nxe1100.c - mfd core driver for the NXE1100
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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>

/* nexell soc headers */
#include <mach/platform.h>
#include <nxe1100.h>
#include <nxe1100-private.h>


/*
 * Debug
 */
#if (0)
#define DBGOUT(msg...)		{ printk("nxe1100-i2c: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

static struct mfd_cell nxe1100_devs[] = {
	{
		.name = "nxe1100-pmic",
	}, {
		.name = "nxe1100-battery",
	},
};

int nxe1100_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct nxe1100_dev *nxe1100 = i2c_get_clientdata(i2c);
	struct i2c_msg msg[2];
	u8  data = reg;
	int ret = 0;

	DBGOUT("%s\n", __func__);

	if (!i2c->adapter)
		return -ENODEV;

	msg[0].addr  = i2c->addr;
	msg[0].flags = 0;
	msg[0].len   = 1;
	msg[0].buf   = &data;

	msg[1].addr  = i2c->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = 1;
	msg[1].buf   = &data;

	mutex_lock(&nxe1100->iolock);
	ret = i2c_transfer(i2c->adapter, msg, 2);
	mutex_unlock(&nxe1100->iolock);

	*dest = data;
	return 0;
}
EXPORT_SYMBOL(nxe1100_read_reg);

int nxe1100_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct nxe1100_dev *nxe1100 = i2c_get_clientdata(i2c);
	struct i2c_msg msg[2];
	int ret = 0;

	DBGOUT("%s\n", __func__);

	if (!i2c->adapter)
		return -ENODEV;

	msg[0].addr  = i2c->addr;
	msg[0].flags = 0;
	msg[0].len   = 1;
	msg[0].buf   = &reg;

	msg[1].addr  = i2c->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = count;
	msg[1].buf   = buf;

	mutex_lock(&nxe1100->iolock);
	ret = i2c_transfer(i2c->adapter, msg, 2);
	mutex_unlock(&nxe1100->iolock);

	return ret;
}
EXPORT_SYMBOL(nxe1100_bulk_read);

int nxe1100_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct nxe1100_dev *nxe1100 = i2c_get_clientdata(i2c);
	struct i2c_msg msg[1];
	u_char *cache	= nxe1100->cache_reg;
	u8		data[2] = { reg, value };
	int		ret;

	DBGOUT("%s\n", __func__);

	if (!i2c->adapter)
		return -ENODEV;

	msg->addr  = i2c->addr;
	msg->flags = 0;
	msg->len   = 2;
	msg->buf   = data;

	mutex_lock(&nxe1100->iolock);
	ret = i2c_transfer(i2c->adapter, msg, 1);
	if (ret >= 0)
		cache[reg] = value;
	mutex_unlock(&nxe1100->iolock);

	return ret;
}
EXPORT_SYMBOL(nxe1100_write_reg);
#define NXE1100_BULK_WRITE_COUNT_MAX	(10)
int nxe1100_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct nxe1100_dev *nxe1100 = i2c_get_clientdata(i2c);
	struct i2c_msg msg[1];
	u_char *cache	= nxe1100->cache_reg;
	u8		data[NXE1100_BULK_WRITE_COUNT_MAX + 1];
	int		i, ret = 0;

	DBGOUT("%s\n", __func__);

	if (!i2c->adapter)
		return -ENODEV;

	if (count > NXE1100_BULK_WRITE_COUNT_MAX)
		count = NXE1100_BULK_WRITE_COUNT_MAX;

	data[0] = reg;
	for (i = 0; i < count; i++) {
		data[i + 1]	= buf[i];
	}

	msg->addr  = i2c->addr;
	msg->flags = 0;
	msg->len   = count + 1;
	msg->buf   = data;

	mutex_lock(&nxe1100->iolock);
	ret = i2c_transfer(i2c->adapter, msg, 1);
	mutex_unlock(&nxe1100->iolock);

	if (ret < 0)
		return ret;

	for (i = 0; i < count; i++) {
		cache[reg + i] = buf[i];
	}

	return 0;
}
EXPORT_SYMBOL(nxe1100_bulk_write);

int nxe1100_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	int ret;
	u8	data;

	DBGOUT("%s\n", __func__);

	ret = nxe1100_read_reg(i2c, reg, &data);
	if (ret == 0) {
		u8 old_val = data & 0xff;
		u8 new_val = (val & mask) | (old_val & (~mask));
		ret = nxe1100_write_reg(i2c, reg, new_val);
	}
	return ret;
}
EXPORT_SYMBOL(nxe1100_update_reg);

static int nxe1100_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct nxe1100_pdata *pdata = i2c->dev.platform_data;
	struct nxe1100_dev *nxe1100;
	int ret = 0;

	DBGOUT("%s\n", __func__);

	nxe1100 = kzalloc(sizeof(struct nxe1100_dev), GFP_KERNEL);
	if (nxe1100 == NULL)
	{
		printk(KERN_ERR "Fail: no memory for nxe1000 parameters !!!\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, nxe1100);
	nxe1100->dev = &i2c->dev;
	nxe1100->i2c = i2c;
	nxe1100->irq = i2c->irq;
	nxe1100->type = id->driver_data;
#if 0
	if (pdata) {
		nxe1100->ono = pdata->ono;
		nxe1100->irq_base = pdata->irq_base;
		nxe1100->wakeup = pdata->wakeup;
	}
#else
	
	if (!pdata)
		goto err;

	nxe1100->gpio_eint = pdata->gpio_eint;
	nxe1100->irq_base = pdata->irq_base;
	nxe1100->ono = pdata->ono;
#endif

	mutex_init(&nxe1100->iolock);

	pm_runtime_set_active(nxe1100->dev);

	nxe1100_irq_init(nxe1100);

#if 0
	switch (id->driver_data) {
	case TYPE_NXE1100:
		ret = mfd_add_devices(nxe1100->dev, -1,
				nxe1100_devs, ARRAY_SIZE(nxe1100_devs),
				NULL, 0);
		break;
	default:
		ret = -EINVAL;
	}
#else

	ret = mfd_add_devices(nxe1100->dev, -1,
			nxe1100_devs, ARRAY_SIZE(nxe1100_devs),
			NULL, 0);
#endif

	if (ret < 0)
		goto err_mfd;

	device_init_wakeup(nxe1100->dev, nxe1100->wakeup);

	return ret;

err_mfd:
	mfd_remove_devices(nxe1100->dev);

err:
	nxe1100_irq_exit(nxe1100);
	kfree(nxe1100);
	return ret;
}

static int nxe1100_i2c_remove(struct i2c_client *i2c)
{
	struct nxe1100_dev *nxe1100 = i2c_get_clientdata(i2c);

	DBGOUT("%s\n", __func__);

	mfd_remove_devices(nxe1100->dev);
	nxe1100_irq_exit(nxe1100);
	kfree(nxe1100);

	return 0;
}

static const struct i2c_device_id nxe1100_i2c_id[] = {
	{ "nxe1100", TYPE_NXE1100 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nxe1100_i2c_id);

static int nxe1100_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct nxe1100_dev *nxe1100 = i2c_get_clientdata(i2c);

	DBGOUT("%s\n", __func__);

	if (device_may_wakeup(dev))
		irq_set_irq_wake(nxe1100->irq, 1);
	return 0;
}

static int nxe1100_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct nxe1100_dev *nxe1100 = i2c_get_clientdata(i2c);

	DBGOUT("%s\n", __func__);

	if (device_may_wakeup(dev))
		irq_set_irq_wake(nxe1100->irq, 0);
	/*
	 * If IRQ registers are not "read & clear"
	 * when it's set during sleep, the interrupt becomes
	 * disabled.
	 */
	return nxe1100_irq_resume(i2c_get_clientdata(i2c));
}

struct nxe1100_reg_dump {
	u8	addr;
	u8	val;
};
#define SAVE_ITEM(x)	{ .addr = (x), .val = 0x0, }
static struct nxe1100_reg_dump nxe1100_dump[] = {
	SAVE_ITEM(NXE1100_REG_DC1CTL),
	SAVE_ITEM(NXE1100_REG_DC1CTL2),
	SAVE_ITEM(NXE1100_REG_DC2CTL),
	SAVE_ITEM(NXE1100_REG_DC2CTL2),
	SAVE_ITEM(NXE1100_REG_DC3CTL),
	SAVE_ITEM(NXE1100_REG_DC3CTL2),
	SAVE_ITEM(NXE1100_REG_DC1DAC),
	SAVE_ITEM(NXE1100_REG_DC2DAC),
	SAVE_ITEM(NXE1100_REG_DC3DAC),
	SAVE_ITEM(NXE1100_REG_LDOEN1),
	SAVE_ITEM(NXE1100_REG_LDOEN2),
	SAVE_ITEM(NXE1100_REG_LDO1DAC),
	SAVE_ITEM(NXE1100_REG_LDO2DAC),
	SAVE_ITEM(NXE1100_REG_LDO3DAC),
	SAVE_ITEM(NXE1100_REG_LDO4DAC),
	SAVE_ITEM(NXE1100_REG_LDO5DAC),
};
/* Save registers before hibernation */
static int nxe1100_freeze(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	int i;

	DBGOUT("%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(nxe1100_dump); i++)
		nxe1100_read_reg(i2c, nxe1100_dump[i].addr,
				&nxe1100_dump[i].val);

	return 0;
}

/* Restore registers after hibernation */
static int nxe1100_restore(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	int i;

	DBGOUT("%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(nxe1100_dump); i++)
		nxe1100_write_reg(i2c, nxe1100_dump[i].addr,
				nxe1100_dump[i].val);

	return 0;
}

static const struct dev_pm_ops nxe1100_pm_ops = {
	.suspend = nxe1100_suspend,
	.resume = nxe1100_resume,
	.freeze = nxe1100_freeze,
	.restore = nxe1100_restore,
};

static struct i2c_driver nxe1100_i2c_driver = {
	.driver = {
		   .name = "nxe1100",
		   .owner = THIS_MODULE,
		   .pm = &nxe1100_pm_ops,
	},
	.probe = nxe1100_i2c_probe,
	.remove = __devexit_p(nxe1100_i2c_remove),
	.id_table = nxe1100_i2c_id,
};

static int __init nxe1100_i2c_init(void)
{
	int ret;

	DBGOUT("%s\n", __func__);

	ret = i2c_add_driver(&nxe1100_i2c_driver);
	if (ret != 0)
		printk("Failed to register nxe1100 I2C driver: %d\n", ret);

	return ret;
}
/* init early so consumer devices can complete system boot */
subsys_initcall(nxe1100_i2c_init);

static void __exit nxe1100_i2c_exit(void)
{
	DBGOUT("%s\n", __func__);

	i2c_del_driver(&nxe1100_i2c_driver);
}
module_exit(nxe1100_i2c_exit);

MODULE_DESCRIPTION("NXE1100 multi-function core driver");
MODULE_AUTHOR("Bongkwan Kook <kook@nexell.co.kr>");
MODULE_LICENSE("GPL");
