/*
 * Interrupt controller support for NXE1100
 *
 * Copyright (C) 2013 Nexell Co,.Ltd.
 *  Bongkwan Kook <kook@nexell.co.kr>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/interrupt.h>

/* nexell soc headers */
#include <mach/platform.h>
#include <mach/soc.h>
#include <nxe1100.h>
#include <nxe1100-private.h>


/*
 * Debug
 */
#if (0)
#define DBGOUT(msg...)		{ printk("nxe1100-irq: " msg); }
#define	NXE1100_REG_DUMP
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

static const u16 nxe1100_mask_reg[] = {
	[NXE1100_IRQGRP_SYSTEM_INT] = NXE1100_REG_TYPE_INVALID,
	[NXE1100_IRQGRP_DCDC_INT]	= NXE1100_REG_TYPE_INTEN | NXE1100_REG_DCIRQ,
	[NXE1100_IRQGRP_ADC_INT1]	= NXE1100_REG_TYPE_INTEN | NXE1100_REG_EN_ADCIR1,
	[NXE1100_IRQGRP_ADC_INT2]	= NXE1100_REG_TYPE_INTEN | NXE1100_REG_EN_ADCIR2,
	[NXE1100_IRQGRP_ADC_INT3]	= NXE1100_REG_TYPE_INTEN | NXE1100_REG_EN_ADCIR3,
	[NXE1100_IRQGRP_GPIO_INT]	= NXE1100_REG_TYPE_INTEN | NXE1100_REG_EN_GPIR,
	[NXE1100_IRQGRP_CHG_INT1]	= NXE1100_REG_CHGCTL_IRFMASK,
	[NXE1100_IRQGRP_CHG_INT2]	= NXE1100_REG_CHGSTAT_IRFMASK1,
	[NXE1100_IRQGRP_CHG_INT3]	= NXE1100_REG_CHGSTAT_IRFMASK2,
	[NXE1100_IRQGRP_CHG_INT4]	= NXE1100_REG_CHGERR_IRFMASK,
	[NXE1100_IRQGRP_FG_INT]		= NXE1100_REG_TYPE_INVALID,
};

static const u16 nxe1100_pend_reg[] = {
	[NXE1100_IRQGRP_SYSTEM_INT] = NXE1100_REG_TYPE_INVALID,
	[NXE1100_IRQGRP_DCDC_INT]	= NXE1100_REG_TYPE_INTEN | NXE1100_REG_DCIRQ,
	[NXE1100_IRQGRP_ADC_INT1]	= NXE1100_REG_TYPE_INTEN | NXE1100_REG_EN_ADCIR1,
	[NXE1100_IRQGRP_ADC_INT2]	= NXE1100_REG_TYPE_INTEN | NXE1100_REG_EN_ADCIR2,
	[NXE1100_IRQGRP_ADC_INT3]	= NXE1100_REG_TYPE_INTEN | NXE1100_REG_EN_ADCIR3,
	[NXE1100_IRQGRP_GPIO_INT]	= NXE1100_REG_TYPE_INTEN | NXE1100_REG_EN_GPIR,
	[NXE1100_IRQGRP_CHG_INT1]	= NXE1100_REG_CHGCTL_IRR,
	[NXE1100_IRQGRP_CHG_INT2]	= NXE1100_REG_CHGSTAT_IRR1,
	[NXE1100_IRQGRP_CHG_INT3]	= NXE1100_REG_CHGSTAT_IRR2,
	[NXE1100_IRQGRP_CHG_INT4]	= NXE1100_REG_CHGERR_IRR,
	[NXE1100_IRQGRP_FG_INT]		= NXE1100_REG_TYPE_INVALID,
};

struct nxe1100_irq_data {
	int mask;
	enum nxe1100_irq_source group;
};

#define DECLARE_IRQ(idx, _group, _mask)		\
	[(idx)] = { .group = (_group), .mask = (_mask) }
static const struct nxe1100_irq_data nxe1100_irqs[] = {
	DECLARE_IRQ(NXE1100_IRQ_DCDC_DC3LIM,	NXE1100_IRQGRP_DCDC_INT,	1 << 2),
	DECLARE_IRQ(NXE1100_IRQ_DCDC_DC2LIM,	NXE1100_IRQGRP_DCDC_INT,	1 << 1),
	DECLARE_IRQ(NXE1100_IRQ_DCDC_DC1LIM,	NXE1100_IRQGRP_DCDC_INT,	1 << 0),

	DECLARE_IRQ(NXE1100_IRQ_ADC1_AIN0L,		NXE1100_IRQGRP_ADC_INT1,	1 << 7),
	DECLARE_IRQ(NXE1100_IRQ_ADC1_AIN1L,		NXE1100_IRQGRP_ADC_INT1,	1 << 6),
	DECLARE_IRQ(NXE1100_IRQ_ADC1_VTHML,		NXE1100_IRQGRP_ADC_INT1,	1 << 5),
	DECLARE_IRQ(NXE1100_IRQ_ADC1_VSYSL,		NXE1100_IRQGRP_ADC_INT1,	1 << 4),
	DECLARE_IRQ(NXE1100_IRQ_ADC1_VUSBL,		NXE1100_IRQGRP_ADC_INT1,	1 << 3),
	DECLARE_IRQ(NXE1100_IRQ_ADC1_VADPL,		NXE1100_IRQGRP_ADC_INT1,	1 << 2),
	DECLARE_IRQ(NXE1100_IRQ_ADC1_VBATL,		NXE1100_IRQGRP_ADC_INT1,	1 << 1),
	DECLARE_IRQ(NXE1100_IRQ_ADC1_ILIML,		NXE1100_IRQGRP_ADC_INT1,	1 << 0),

	DECLARE_IRQ(NXE1100_IRQ_ADC2_AIN0H,		NXE1100_IRQGRP_ADC_INT2,	1 << 7),
	DECLARE_IRQ(NXE1100_IRQ_ADC2_AIN1H,		NXE1100_IRQGRP_ADC_INT2,	1 << 6),
	DECLARE_IRQ(NXE1100_IRQ_ADC2_VTHMH,		NXE1100_IRQGRP_ADC_INT2,	1 << 5),
	DECLARE_IRQ(NXE1100_IRQ_ADC2_VSYSH,		NXE1100_IRQGRP_ADC_INT2,	1 << 4),
	DECLARE_IRQ(NXE1100_IRQ_ADC2_VUSBH,		NXE1100_IRQGRP_ADC_INT2,	1 << 3),
	DECLARE_IRQ(NXE1100_IRQ_ADC2_VADPH,		NXE1100_IRQGRP_ADC_INT2,	1 << 2),
	DECLARE_IRQ(NXE1100_IRQ_ADC2_VBATH,		NXE1100_IRQGRP_ADC_INT2,	1 << 1),
	DECLARE_IRQ(NXE1100_IRQ_ADC2_ILIMH,		NXE1100_IRQGRP_ADC_INT2,	1 << 0),

	DECLARE_IRQ(NXE1100_IRQ_ADC3_END,		NXE1100_IRQGRP_ADC_INT3,	1 << 0),

	DECLARE_IRQ(NXE1100_IRQ_GPIO_GP03,		NXE1100_IRQGRP_GPIO_INT,	1 << 3),
	DECLARE_IRQ(NXE1100_IRQ_GPIO_GP02,		NXE1100_IRQGRP_GPIO_INT,	1 << 2),
	DECLARE_IRQ(NXE1100_IRQ_GPIO_GP01,		NXE1100_IRQGRP_GPIO_INT,	1 << 1),
	DECLARE_IRQ(NXE1100_IRQ_GPIO_GP00,		NXE1100_IRQGRP_GPIO_INT,	1 << 0),

	DECLARE_IRQ(NXE1100_IRQ_CHGCTRL_RDSTATESHIFT,	NXE1100_IRQGRP_CHG_INT1,	1 << 6),
	DECLARE_IRQ(NXE1100_IRQ_CHGCTRL_WVUSBS,		NXE1100_IRQGRP_CHG_INT1,	1 << 5),
	DECLARE_IRQ(NXE1100_IRQ_CHGCTRL_WVADPS,		NXE1100_IRQGRP_CHG_INT1,	1 << 4),
	DECLARE_IRQ(NXE1100_IRQ_CHGCTRL_VUSBLVS,	NXE1100_IRQGRP_CHG_INT1,	1 << 3),
	DECLARE_IRQ(NXE1100_IRQ_CHGCTRL_VADPLVS,	NXE1100_IRQGRP_CHG_INT1,	1 << 2),
	DECLARE_IRQ(NXE1100_IRQ_CHGCTRL_VUSBDETS,	NXE1100_IRQGRP_CHG_INT1,	1 << 1),
	DECLARE_IRQ(NXE1100_IRQ_CHGCTRL_VADPDETS,	NXE1100_IRQGRP_CHG_INT1,	1 << 0),

	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_BTEMPJTA4,	NXE1100_IRQGRP_CHG_INT2,	1 << 7),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_BTEMPJTA3,	NXE1100_IRQGRP_CHG_INT2,	1 << 6),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_BTEMPJTA2,	NXE1100_IRQGRP_CHG_INT2,	1 << 5),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_BTEMPJTA1,	NXE1100_IRQGRP_CHG_INT2,	1 << 4),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_SLPMODE,		NXE1100_IRQGRP_CHG_INT2,	1 << 3),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_BATOPEN,		NXE1100_IRQGRP_CHG_INT2,	1 << 2),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_CHGCMP,		NXE1100_IRQGRP_CHG_INT2,	1 << 1),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_ONCHG,		NXE1100_IRQGRP_CHG_INT2,	1 << 0),

	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_OSCMDET,		NXE1100_IRQGRP_CHG_INT3,	1 << 7),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_OSCFDET3,	NXE1100_IRQGRP_CHG_INT3,	1 << 6),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_OSCFDET2,	NXE1100_IRQGRP_CHG_INT3,	1 << 5),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_OSCFDET1,	NXE1100_IRQGRP_CHG_INT3,	1 << 4),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_POOR_CHGCUR,	NXE1100_IRQGRP_CHG_INT3,	1 << 3),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_ICRVS,		NXE1100_IRQGRP_CHG_INT3,	1 << 2),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_VOLTERM,		NXE1100_IRQGRP_CHG_INT3,	1 << 1),
	DECLARE_IRQ(NXE1100_IRQ_CHGSTS_CURTERM,		NXE1100_IRQGRP_CHG_INT3,	1 << 0),

	DECLARE_IRQ(NXE1100_IRQ_CHGERR_VUSBOVS,		NXE1100_IRQGRP_CHG_INT4,	1 << 7),
	DECLARE_IRQ(NXE1100_IRQ_CHGERR_VADPOVS,		NXE1100_IRQGRP_CHG_INT4,	1 << 6),
	DECLARE_IRQ(NXE1100_IRQ_CHGERR_RTIMOV,		NXE1100_IRQGRP_CHG_INT4,	1 << 5),
	DECLARE_IRQ(NXE1100_IRQ_CHGERR_TTIMOV,		NXE1100_IRQGRP_CHG_INT4,	1 << 4),
	DECLARE_IRQ(NXE1100_IRQ_CHGERR_VBATOV,		NXE1100_IRQGRP_CHG_INT4,	1 << 3),
	DECLARE_IRQ(NXE1100_IRQ_CHGERR_BTEMPERR,	NXE1100_IRQGRP_CHG_INT4,	1 << 2),
	DECLARE_IRQ(NXE1100_IRQ_CHGERR_DIEERR,		NXE1100_IRQGRP_CHG_INT4,	1 << 1),
	DECLARE_IRQ(NXE1100_IRQ_CHGERR_DIEOFF,		NXE1100_IRQGRP_CHG_INT4,	1 << 0),
};

static void nxe1100_irq_lock(struct irq_data *data)
{
	struct nxe1100_dev *nxe1100 = irq_get_chip_data(data->irq);

	DBGOUT("%s\n", __func__);

	mutex_lock(&nxe1100->irqlock);
}

static void nxe1100_irq_sync_unlock(struct irq_data *data)
{
	struct nxe1100_dev *nxe1100 = irq_get_chip_data(data->irq);
	struct i2c_client *i2c;
	u8	gpio_irq_type = 0;
	int i;

	DBGOUT("%s\n", __func__);

	i2c = nxe1100->i2c;

	for (i = 0; i < ARRAY_SIZE(nxe1100->gpio_mode); i++) {
		if (nxe1100->gpio_mode[i]) {
			gpio_irq_type |= (nxe1100->gpio_mode[i] & NXE1100_VAL_GPEDGE1_EDGE_MASK) << (i * 2);
			nxe1100->gpio_mode[i] = 0;
		}
	}
	if (gpio_irq_type) {
		nxe1100_write_reg(i2c, NXE1100_REG_GPEDGE1, gpio_irq_type);
	}

	for (i = 0; i < NXE1100_IRQ_GROUP_NR; i++) {
		u16 mask_reg = (nxe1100_mask_reg[i] & 0xFF00);

		if (mask_reg == NXE1100_REG_TYPE_INVALID ||
				IS_ERR_OR_NULL(i2c))
			continue;

		nxe1100->irq_masks_cache[i] = nxe1100->irq_masks_cur[i];

		if (mask_reg == NXE1100_REG_TYPE_INTEN)
		{
			nxe1100_write_reg(i2c, (u8)(nxe1100_mask_reg[i] & 0xFF),
					~nxe1100->irq_masks_cur[i]);
		}
		else
		{
			nxe1100_write_reg(i2c, (u8)(nxe1100_mask_reg[i] & 0xFF),
					nxe1100->irq_masks_cur[i]);
		}
	}

	mutex_unlock(&nxe1100->irqlock);
}

static inline struct nxe1100_irq_data *
irq_to_nxe1100_irq(struct nxe1100_dev *nxe1100, int irq)
{
	DBGOUT("%s\n", __func__);

	return (struct nxe1100_irq_data *)&nxe1100_irqs[irq - nxe1100->irq_base];
}

static void nxe1100_irq_mask(struct irq_data *data)
{
	struct nxe1100_dev *nxe1100 = irq_get_chip_data(data->irq);
	struct nxe1100_irq_data *irq_data = irq_to_nxe1100_irq(nxe1100,
							       data->irq);

	DBGOUT("%s\n", __func__);

	nxe1100->irq_masks_cur[irq_data->group] |= irq_data->mask;
}

static void nxe1100_irq_unmask(struct irq_data *data)
{
	struct nxe1100_dev *nxe1100 = irq_get_chip_data(data->irq);
	struct nxe1100_irq_data *irq_data = irq_to_nxe1100_irq(nxe1100,
							       data->irq);

	DBGOUT("%s\n", __func__);

	nxe1100->irq_masks_cur[irq_data->group] &= ~irq_data->mask;
}

static int nxe1100_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct nxe1100_dev *nxe1100 = irq_get_chip_data(data->irq);
	int irq;

	irq = data->irq - nxe1100->irq_base;

	if (irq < NXE1100_IRQ_GPIO_GP00 || irq > NXE1100_IRQ_GPIO_GP03) {
		/* Ignore internal-only IRQs */
		if (irq >= 0 && irq < NXE1100_IRQ_NR)
			return 0;
		else
			return -EINVAL;
	}

	/* Rebase the IRQ into the GPIO range so we've got a sensible array
	 * index.
	 */
	irq -= NXE1100_IRQ_GPIO_GP00;

	/* We set the high bit to flag that we need an update; don't
	 * do the update here as we can be called with the bus lock
	 * held.
	 */
	switch (type) {
	case IRQ_TYPE_EDGE_BOTH:
		nxe1100->gpio_mode[irq] = 0x10 | NXE1100_VAL_GPEDGE1_BOTH_EDGE;
		break;
	case IRQ_TYPE_EDGE_RISING:
		nxe1100->gpio_mode[irq] = 0x10 | NXE1100_VAL_GPEDGE1_RISING_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		nxe1100->gpio_mode[irq] = 0x10 | NXE1100_VAL_GPEDGE1_FALLING_EDGE;
		break;
	case IRQ_TYPE_LEVEL_MASK:
		nxe1100->gpio_mode[irq] = 0x10 | NXE1100_VAL_GPEDGE1_LEVEL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct irq_chip nxe1100_irq_chip = {
	.name					= "nxe1100",
	.irq_bus_lock			= nxe1100_irq_lock,
	.irq_bus_sync_unlock	= nxe1100_irq_sync_unlock,
	.irq_enable 			= nxe1100_irq_unmask,
	.irq_disable			= nxe1100_irq_mask,
	.irq_set_type			= nxe1100_irq_set_type,
};

#define NXE1100_IRQSRC_SYSTEM	(1 << 0)
#define NXE1100_IRQSRC_DCDC		(1 << 1)
#define NXE1100_IRQSRC_ADC		(1 << 3)
#define NXE1100_IRQSRC_GPIO		(1 << 4)
#define NXE1100_IRQSRC_CHG		(1 << 6)
#define NXE1100_IRQSRC_FG		(1 << 7)
static irqreturn_t nxe1100_irq_thread(int irq, void *data)
{
	struct nxe1100_dev *nxe1100 = data;
	u8 *status_regs;
	u8	irq_src;
	int ret;
	int i;

//	DBGOUT("%s\n", __func__);

	ret = nxe1100_read_reg(nxe1100->i2c, NXE1100_REG_INTMON, &irq_src);
	if (ret < 0) {
		dev_err(nxe1100->dev, "Failed to read interrupt source: ret[%d], irq_src[0x%02x]\n",
				ret, irq_src);
		return IRQ_NONE;
	}

	status_regs = nxe1100->status_regs;
	memset(status_regs, 0x00, NXE1100_IRQ_GROUP_NR);

	if (!irq_src)
	{
		nxe1100_read_reg(nxe1100->i2c, NXE1100_REG_CHGSTATE,
				&status_regs[NXE1100_IRQGRP_FG_INT]);
		status_regs[NXE1100_IRQGRP_CHG_INT1] = (1 << NXE1100_POS_CHGCTRL_MONI_RDSTATESHIFT);
		printk("chg state [%02xh - 0x%02x]\n", NXE1100_REG_CHGSTATE, status_regs[NXE1100_IRQGRP_FG_INT]);
	}

	if (irq_src & NXE1100_IRQSRC_SYSTEM) {
		status_regs[NXE1100_IRQGRP_SYSTEM_INT] = 0;
	}
	if (irq_src & NXE1100_IRQSRC_DCDC) {
		nxe1100_read_reg(nxe1100->i2c, NXE1100_REG_DCIRQ,
				&status_regs[NXE1100_IRQGRP_DCDC_INT]);
	}
	if (irq_src & NXE1100_IRQSRC_ADC) {
		/* ADC INT1 ~ INT3 */
		nxe1100_bulk_read(nxe1100->i2c, NXE1100_REG_IR_ADC1, 3,
				&status_regs[NXE1100_IRQGRP_ADC_INT1]);
	}
	if (irq_src & NXE1100_IRQSRC_GPIO) {
		u8 gpio_edge_info, gpio_intr_info[3];

		/* GPIO Rising & Falling Interrupt */
		nxe1100_read_reg(nxe1100->i2c, NXE1100_REG_GPEDGE1,
				&gpio_edge_info);
		nxe1100_bulk_read(nxe1100->i2c, NXE1100_REG_EN_GPIR, 3,
				gpio_intr_info);

		for (i = 0; i < NXE1100_NUM_GPIO; i++) {
			bool interrupt = false;

			switch (gpio_edge_info & (NXE1100_GPIO_INT_MASK << (i * 2))) {
			case NXE1100_GPIO_INT_BOTH:
				if (gpio_intr_info[0] & (gpio_intr_info[1] | gpio_intr_info[2]))
					interrupt = true;
				break;
			case NXE1100_GPIO_INT_RISE:
				if (gpio_intr_info[0] & gpio_intr_info[1])
					interrupt = true;
				break;
			case NXE1100_GPIO_INT_FALL:
				if (gpio_intr_info[0] & gpio_intr_info[2])
					interrupt = true;
				break;
			default:
				break;
			}

			if (interrupt) {
				status_regs[NXE1100_IRQGRP_GPIO_INT] |= (1 << i);
			}
		}
	}
	if (irq_src & NXE1100_IRQSRC_CHG) {
		nxe1100_bulk_read(nxe1100->i2c, NXE1100_REG_CHGCTL_MONI, 4,
				&status_regs[NXE1100_IRQGRP_CHG_INT1]);
	}
	if (irq_src & NXE1100_IRQSRC_FG) {
		status_regs[NXE1100_IRQGRP_FG_INT] = 0;
	}

	/* Report */
	for (i = 0; i < NXE1100_IRQ_NR; i++) {
		if (status_regs[nxe1100_irqs[i].group] & nxe1100_irqs[i].mask) {
			handle_nested_irq(nxe1100->irq_base + i);
		}
	}

	/* Pending clear */
	for (i = 0; i < NXE1100_IRQ_GROUP_NR; i++)
	{
		if (nxe1100_mask_reg[i] == NXE1100_REG_TYPE_INVALID)
			continue;

		if (status_regs[i])
		{
			if ((nxe1100_mask_reg[i] & 0xFF00) == NXE1100_REG_TYPE_INTEN)
			{
				nxe1100_update_reg(nxe1100->i2c, (u8)(nxe1100_mask_reg[i] & 0xFF), ~status_regs[i], status_regs[i]);
			}
			else
			{
//				nxe1100_update_reg(nxe1100->i2c, (u8)(nxe1100_mask_reg[i] & 0xFF), status_regs[i], status_regs[i]);
				nxe1100_write_reg(nxe1100->i2c, (u8)(nxe1100_pend_reg[i] & 0xFF), status_regs[i]);
			}
		}
	}

	return IRQ_HANDLED;
}

int nxe1100_irq_resume(struct nxe1100_dev *nxe1100)
{
	DBGOUT("%s\n", __func__);

	if (nxe1100->irq && nxe1100->irq_base)
		nxe1100_irq_thread(nxe1100->irq_base, nxe1100);
	return 0;
}

int nxe1100_irq_init(struct nxe1100_dev *nxe1100)
{
	int i;
	int cur_irq;
	int ret;

	DBGOUT("%s\n", __func__);

	if (!nxe1100->irq) {
		dev_warn(nxe1100->dev,
			 "No interrupt specified, no interrupts\n");
		nxe1100->irq_base = 0;
		return 0;
	}

	if (!nxe1100->irq_base) {
		dev_err(nxe1100->dev,
			"No interrupt base specified, no interrupts\n");
		return 0;
	}

	mutex_init(&nxe1100->irqlock);

	/* Disable top level interrupts */
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_INTEN,		0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_DCIREN,		0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_EN_ADCIR1,	0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_EN_ADCIR2,	0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_EN_ADCIR3,	0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_EN_GPIR,	0x00);

	/* Mask the individual interrupt sources */
	for (i = 0; i < NXE1100_IRQ_GROUP_NR; i++) {
		nxe1100->irq_masks_cur[i] = 0xff;
		nxe1100->irq_masks_cache[i] = 0xff;

		if (nxe1100_mask_reg[i] == NXE1100_REG_TYPE_INVALID)
			continue;

		nxe1100_write_reg(nxe1100->i2c, (u8)(nxe1100_mask_reg[i] & 0xFF), 0xff);
	}

	/* Pending clear */
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_DCIRQ,		0x00);

	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_IR_ADC1,	0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_IR_ADC2,	0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_IR_ADC3,	0x00);

	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_IR_GPR,		0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_IR_GPF,		0x00);

	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_CHGCTL_IRR,		0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_CHGSTAT_IRR1,	0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_CHGSTAT_IRR2,	0x00);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_CHGERR_IRR,		0x00);

	/* Register with genirq */
	for (cur_irq = nxe1100->irq_base;
		cur_irq < (NXE1100_IRQ_NR + nxe1100->irq_base);
		cur_irq++) {
		irq_set_chip_data(cur_irq, nxe1100);
		irq_set_chip_and_handler(cur_irq, &nxe1100_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);

		/* ARM needs us to explicitly flag the IRQ as valid
		 * and will set them noprobe when we do so. */
#ifdef CONFIG_ARM
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		irq_set_noprobe(cur_irq);
#endif
	}

	if (nxe1100->irq) {
		ret = request_threaded_irq(nxe1100->irq, NULL, nxe1100_irq_thread,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   "nxe1100-irq", nxe1100);
		if (ret) {
			dev_err(nxe1100->dev, "Failed to request IRQ %d: %d\n",
				nxe1100->irq, ret);
			return ret;
		}
	} else {
		dev_warn(nxe1100->dev,
			 "No interrupt specified - functionality limited\n");
	}

	/* Interrup enable */
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_EN_ADCIR1,	0xff);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_EN_ADCIR2,	0xff);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_EN_ADCIR3,	0xff);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_EN_GPIR,	0xff);
//	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_DCIREN, 	0xff);
	nxe1100_write_reg(nxe1100->i2c, NXE1100_REG_INTEN,		0xdb);

	if (!nxe1100->ono)
		return 0;

	ret = request_threaded_irq(nxe1100->ono, NULL, nxe1100_irq_thread,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING |
				   IRQF_ONESHOT, "nxe1100-ono", nxe1100);
	if (ret)
		dev_err(nxe1100->dev, "Failed to request ono-IRQ %d: %d\n",
			nxe1100->ono, ret);

	return 0;
}

void nxe1100_irq_exit(struct nxe1100_dev *nxe1100)
{
	DBGOUT("%s\n", __func__);

	if (nxe1100->ono)
		free_irq(nxe1100->ono, nxe1100);

	if (nxe1100->irq)
		free_irq(nxe1100->irq, nxe1100);
}
