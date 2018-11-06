/*
 * Interrupt controller support for NXE2000
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
#include <linux/gpio.h>

/* nexell soc headers */
#include <mach/platform.h>
#include <mach/soc.h>
#include <nxe1999.h>
#include <nxe2000-private.h>


/*
 * Debug
 */
#if (0)
#define DBGOUT(msg...)		{ printk("nxe2000-irq: " msg); }
#define	NXE2000_REG_DUMP
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

static const u16 nxe2000_mask_reg[] = {
	[NXE2000_IRQGRP_SYSTEM_INT] = NXE2000_REG_TYPE_INVALID,
	[NXE2000_IRQGRP_DCDC_INT]	= NXE2000_REG_DCIRQ,
	[NXE2000_IRQGRP_ADC_INT1]	= NXE2000_REG_TYPE_INTEN | NXE2000_REG_EN_ADCIR1,
	[NXE2000_IRQGRP_ADC_INT2]	= NXE2000_REG_TYPE_INTEN | NXE2000_REG_EN_ADCIR2,
	[NXE2000_IRQGRP_ADC_INT3]	= NXE2000_REG_TYPE_INTEN | NXE2000_REG_EN_ADCIR3,
	[NXE2000_IRQGRP_GPIO_INT]	= NXE2000_REG_TYPE_INTEN | NXE2000_REG_EN_GPIR,
	[NXE2000_IRQGRP_CHG_INT1]	= NXE2000_REG_CHGCTL_IRFMASK,
	[NXE2000_IRQGRP_CHG_INT2]	= NXE2000_REG_CHGSTAT_IRFMASK1,
	[NXE2000_IRQGRP_CHG_INT3]	= NXE2000_REG_CHGSTAT_IRFMASK2,
	[NXE2000_IRQGRP_CHG_INT4]	= NXE2000_REG_CHGERR_IRFMASK,
	[NXE2000_IRQGRP_FG_INT]		= NXE2000_REG_TYPE_INVALID
};

static const u16 nxe2000_pend_reg[] = {
	[NXE2000_IRQGRP_SYSTEM_INT] = NXE2000_REG_TYPE_INVALID,
	[NXE2000_IRQGRP_DCDC_INT]	= NXE2000_REG_DCIRQ,
	[NXE2000_IRQGRP_ADC_INT1]	= NXE2000_REG_TYPE_INTEN | NXE2000_REG_IR_ADC1,
	[NXE2000_IRQGRP_ADC_INT2]	= NXE2000_REG_TYPE_INTEN | NXE2000_REG_IR_ADC2,
	[NXE2000_IRQGRP_ADC_INT3]	= NXE2000_REG_TYPE_INTEN | NXE2000_REG_IR_ADC3,
	[NXE2000_IRQGRP_GPIO_INT]	= NXE2000_REG_TYPE_INTEN | NXE2000_REG_IR_GPR,
	[NXE2000_IRQGRP_CHG_INT1]	= NXE2000_REG_CHGCTL_IRR,
	[NXE2000_IRQGRP_CHG_INT2]	= NXE2000_REG_CHGSTAT_IRR1,
	[NXE2000_IRQGRP_CHG_INT3]	= NXE2000_REG_CHGSTAT_IRR2,
	[NXE2000_IRQGRP_CHG_INT4]	= NXE2000_REG_CHGERR_IRR,
	[NXE2000_IRQGRP_FG_INT]		= NXE2000_REG_TYPE_INVALID
};

struct nxe2000_irq_data {
	int mask;
	enum nxe2000_irq_source group;
};

#define DECLARE_IRQ(idx, _group, _mask)		\
	[(idx)] = { .group = (_group), .mask = (_mask) }
static const struct nxe2000_irq_data nxe2000_irqs[] = {
	DECLARE_IRQ(NXE2000_IRQ_DCDC_DC1LIM,	NXE2000_IRQGRP_DCDC_INT,	1 << 0),
	DECLARE_IRQ(NXE2000_IRQ_DCDC_DC2LIM,    NXE2000_IRQGRP_DCDC_INT,    1 << 1),
	DECLARE_IRQ(NXE2000_IRQ_DCDC_DC3LIM,    NXE2000_IRQGRP_DCDC_INT,    1 << 2),
	DECLARE_IRQ(NXE2000_IRQ_DCDC_DC4LIM,    NXE2000_IRQGRP_DCDC_INT,    1 << 3),
	DECLARE_IRQ(NXE2000_IRQ_DCDC_DC5LIM,    NXE2000_IRQGRP_DCDC_INT,    1 << 4),

	DECLARE_IRQ(NXE2000_IRQ_ADC1_AIN0L,		NXE2000_IRQGRP_ADC_INT1,	1 << 7),
	DECLARE_IRQ(NXE2000_IRQ_ADC1_AIN1L,		NXE2000_IRQGRP_ADC_INT1,	1 << 6),
	DECLARE_IRQ(NXE2000_IRQ_ADC1_VTHML,		NXE2000_IRQGRP_ADC_INT1,	1 << 5),
	DECLARE_IRQ(NXE2000_IRQ_ADC1_VSYSL,		NXE2000_IRQGRP_ADC_INT1,	1 << 4),
	DECLARE_IRQ(NXE2000_IRQ_ADC1_VUSBL,		NXE2000_IRQGRP_ADC_INT1,	1 << 3),
	DECLARE_IRQ(NXE2000_IRQ_ADC1_VADPL,		NXE2000_IRQGRP_ADC_INT1,	1 << 2),
	DECLARE_IRQ(NXE2000_IRQ_ADC1_VBATL,		NXE2000_IRQGRP_ADC_INT1,	1 << 1),
	DECLARE_IRQ(NXE2000_IRQ_ADC1_ILIML,		NXE2000_IRQGRP_ADC_INT1,	1 << 0),

	DECLARE_IRQ(NXE2000_IRQ_ADC2_AIN0H,		NXE2000_IRQGRP_ADC_INT2,	1 << 7),
	DECLARE_IRQ(NXE2000_IRQ_ADC2_AIN1H,		NXE2000_IRQGRP_ADC_INT2,	1 << 6),
	DECLARE_IRQ(NXE2000_IRQ_ADC2_VTHMH,		NXE2000_IRQGRP_ADC_INT2,	1 << 5),
	DECLARE_IRQ(NXE2000_IRQ_ADC2_VSYSH,		NXE2000_IRQGRP_ADC_INT2,	1 << 4),
	DECLARE_IRQ(NXE2000_IRQ_ADC2_VUSBH,		NXE2000_IRQGRP_ADC_INT2,	1 << 3),
	DECLARE_IRQ(NXE2000_IRQ_ADC2_VADPH,		NXE2000_IRQGRP_ADC_INT2,	1 << 2),
	DECLARE_IRQ(NXE2000_IRQ_ADC2_VBATH,		NXE2000_IRQGRP_ADC_INT2,	1 << 1),
	DECLARE_IRQ(NXE2000_IRQ_ADC2_ILIMH,		NXE2000_IRQGRP_ADC_INT2,	1 << 0),

	DECLARE_IRQ(NXE2000_IRQ_ADC3_END,		NXE2000_IRQGRP_ADC_INT3,	1 << 0),

	DECLARE_IRQ(NXE2000_IRQ_GPIO_GP04,		NXE2000_IRQGRP_GPIO_INT,	1 << 4),
	DECLARE_IRQ(NXE2000_IRQ_GPIO_GP03,		NXE2000_IRQGRP_GPIO_INT,	1 << 3),
	DECLARE_IRQ(NXE2000_IRQ_GPIO_GP02,		NXE2000_IRQGRP_GPIO_INT,	1 << 2),
	DECLARE_IRQ(NXE2000_IRQ_GPIO_GP01,		NXE2000_IRQGRP_GPIO_INT,	1 << 1),
	DECLARE_IRQ(NXE2000_IRQ_GPIO_GP00,		NXE2000_IRQGRP_GPIO_INT,	1 << 0),

	DECLARE_IRQ(NXE2000_IRQ_CHGCTRL_RDSTATESHIFT,	NXE2000_IRQGRP_CHG_INT1,	1 << 6),
	DECLARE_IRQ(NXE2000_IRQ_CHGCTRL_WVUSBS,		NXE2000_IRQGRP_CHG_INT1,	1 << 5),
	DECLARE_IRQ(NXE2000_IRQ_CHGCTRL_WVADPS,		NXE2000_IRQGRP_CHG_INT1,	1 << 4),
	DECLARE_IRQ(NXE2000_IRQ_CHGCTRL_VUSBLVS,	NXE2000_IRQGRP_CHG_INT1,	1 << 3),
	DECLARE_IRQ(NXE2000_IRQ_CHGCTRL_VADPLVS,	NXE2000_IRQGRP_CHG_INT1,	1 << 2),
	DECLARE_IRQ(NXE2000_IRQ_CHGCTRL_VUSBDETS,	NXE2000_IRQGRP_CHG_INT1,	1 << 1),
	DECLARE_IRQ(NXE2000_IRQ_CHGCTRL_VADPDETS,	NXE2000_IRQGRP_CHG_INT1,	1 << 0),

	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_BTEMPJTA4,	NXE2000_IRQGRP_CHG_INT2,	1 << 7),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_BTEMPJTA3,	NXE2000_IRQGRP_CHG_INT2,	1 << 6),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_BTEMPJTA2,	NXE2000_IRQGRP_CHG_INT2,	1 << 5),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_BTEMPJTA1,	NXE2000_IRQGRP_CHG_INT2,	1 << 4),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_SLPMODE,		NXE2000_IRQGRP_CHG_INT2,	1 << 3),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_BATOPEN,		NXE2000_IRQGRP_CHG_INT2,	1 << 2),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_CHGCMP,		NXE2000_IRQGRP_CHG_INT2,	1 << 1),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_ONCHG,		NXE2000_IRQGRP_CHG_INT2,	1 << 0),

	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_OSCMDET,		NXE2000_IRQGRP_CHG_INT3,	1 << 7),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_OSCFDET3,	NXE2000_IRQGRP_CHG_INT3,	1 << 6),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_OSCFDET2,	NXE2000_IRQGRP_CHG_INT3,	1 << 5),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_OSCFDET1,	NXE2000_IRQGRP_CHG_INT3,	1 << 4),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_POOR_CHGCUR,	NXE2000_IRQGRP_CHG_INT3,	1 << 3),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_ICRVS,		NXE2000_IRQGRP_CHG_INT3,	1 << 2),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_VOLTERM,		NXE2000_IRQGRP_CHG_INT3,	1 << 1),
	DECLARE_IRQ(NXE2000_IRQ_CHGSTS_CURTERM,		NXE2000_IRQGRP_CHG_INT3,	1 << 0),

	DECLARE_IRQ(NXE2000_IRQ_CHGERR_VUSBOVS,		NXE2000_IRQGRP_CHG_INT4,	1 << 7),
	DECLARE_IRQ(NXE2000_IRQ_CHGERR_VADPOVS,		NXE2000_IRQGRP_CHG_INT4,	1 << 6),
	DECLARE_IRQ(NXE2000_IRQ_CHGERR_RTIMOV,		NXE2000_IRQGRP_CHG_INT4,	1 << 5),
	DECLARE_IRQ(NXE2000_IRQ_CHGERR_TTIMOV,		NXE2000_IRQGRP_CHG_INT4,	1 << 4),
	DECLARE_IRQ(NXE2000_IRQ_CHGERR_VBATOV,		NXE2000_IRQGRP_CHG_INT4,	1 << 3),
	DECLARE_IRQ(NXE2000_IRQ_CHGERR_BTEMPERR,	NXE2000_IRQGRP_CHG_INT4,	1 << 2),
	DECLARE_IRQ(NXE2000_IRQ_CHGERR_DIEERR,		NXE2000_IRQGRP_CHG_INT4,	1 << 1),
	DECLARE_IRQ(NXE2000_IRQ_CHGERR_DIEOFF,		NXE2000_IRQGRP_CHG_INT4,	1 << 0)
};

static void nxe2000_irq_lock(struct irq_data *data)
{
	struct nxe2000_dev *nxe2000 = irq_get_chip_data(data->irq);

	DBGOUT("%s\n", __func__);

	mutex_lock(&nxe2000->irqlock);
}

static void nxe2000_irq_sync_unlock(struct irq_data *data)
{
	struct nxe2000_dev *nxe2000 = irq_get_chip_data(data->irq);
	struct i2c_client *i2c;
	u8	gpio_irq_type = 0;
	int i;

	DBGOUT("%s\n", __func__);

	i2c = nxe2000->i2c;

	for (i = 0; i < ARRAY_SIZE(nxe2000->gpio_mode); i++) {
		if (nxe2000->gpio_mode[i]) {
			gpio_irq_type |= (nxe2000->gpio_mode[i] & NXE2000_VAL_GPEDGE1_EDGE_MASK) << (i * 2);
			nxe2000->gpio_mode[i] = 0;
		}
	}
	if (gpio_irq_type) {
		nxe2000_write(i2c, NXE2000_REG_GPEDGE1, gpio_irq_type);
	}

	for (i = 0; i < ARRAY_SIZE(nxe2000->irq_masks_cur); i++) {
		u16 mask_reg = (nxe2000_mask_reg[i] & 0xFF00);

		if (mask_reg == NXE2000_REG_TYPE_INVALID ||
				IS_ERR_OR_NULL(i2c))
			continue;

		if (nxe2000->irq_masks_cache[i] != nxe2000->irq_masks_cur[i])
		{      
			if (mask_reg == NXE2000_REG_TYPE_INTEN)
			{
				nxe2000_write(i2c, (u8)(nxe2000_mask_reg[i] & 0xFF),
						~nxe2000->irq_masks_cur[i]);
			}
			else
			{
				nxe2000_write(i2c, (u8)(nxe2000_mask_reg[i] & 0xFF),
						nxe2000->irq_masks_cur[i]);
			}

			nxe2000->irq_masks_cache[i] != nxe2000->irq_masks_cur[i];
		}
	}

	mutex_unlock(&nxe2000->irqlock);
}

static inline struct nxe2000_irq_data *
irq_to_nxe2000_irq(struct nxe2000_dev *nxe2000, int irq)
{
	DBGOUT("%s\n", __func__);

	return (struct nxe2000_irq_data *)&nxe2000_irqs[irq - nxe2000->irq_base];
}

static void nxe2000_irq_mask(struct irq_data *data)
{
	struct nxe2000_dev *nxe2000 = irq_get_chip_data(data->irq);
	struct nxe2000_irq_data *irq_data = irq_to_nxe2000_irq(nxe2000,
							       data->irq);

	DBGOUT("%s\n", __func__);

	nxe2000->irq_masks_cur[irq_data->group] |= irq_data->mask;
}

static void nxe2000_irq_unmask(struct irq_data *data)
{
	struct nxe2000_dev *nxe2000 = irq_get_chip_data(data->irq);
	struct nxe2000_irq_data *irq_data = irq_to_nxe2000_irq(nxe2000,
							       data->irq);

	DBGOUT("%s\n", __func__);

	nxe2000->irq_masks_cur[irq_data->group] &= ~irq_data->mask;
}

static int nxe2000_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct nxe2000_dev *nxe2000 = irq_get_chip_data(data->irq);
	int irq;

	irq = data->irq - nxe2000->irq_base;

	if (irq < NXE2000_IRQ_GPIO_GP00 || irq > NXE2000_IRQ_GPIO_GP03) {
		/* Ignore internal-only IRQs */
		if (irq >= 0 && irq < NXE2000_IRQ_NR)
			return 0;
		else
			return -EINVAL;
	}

	/* Rebase the IRQ into the GPIO range so we've got a sensible array
	 * index.
	 */
	irq -= NXE2000_IRQ_GPIO_GP00;

	/* We set the high bit to flag that we need an update; don't
	 * do the update here as we can be called with the bus lock
	 * held.
	 */
	switch (type) {
	case IRQ_TYPE_EDGE_BOTH:
		nxe2000->gpio_mode[irq] = 0x10 | NXE2000_VAL_GPEDGE1_BOTH_EDGE;
		break;
	case IRQ_TYPE_EDGE_RISING:
		nxe2000->gpio_mode[irq] = 0x10 | NXE2000_VAL_GPEDGE1_RISING_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		nxe2000->gpio_mode[irq] = 0x10 | NXE2000_VAL_GPEDGE1_FALLING_EDGE;
		break;
	case IRQ_TYPE_LEVEL_MASK:
		nxe2000->gpio_mode[irq] = 0x10 | NXE2000_VAL_GPEDGE1_LEVEL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct irq_chip nxe2000_irq_chip = {
	.name					= "nxe2000",
	.irq_mask				= nxe2000_irq_mask,
	.irq_unmask				= nxe2000_irq_unmask,
	.irq_bus_lock			= nxe2000_irq_lock,
	.irq_bus_sync_unlock	= nxe2000_irq_sync_unlock,
//	.irq_enable 			= nxe2000_irq_unmask,
//	.irq_disable			= nxe2000_irq_mask,
	.irq_set_type			= nxe2000_irq_set_type,
};

#define NXE2000_IRQSRC_SYSTEM	(1 << 0)
#define NXE2000_IRQSRC_DCDC		(1 << 1)
#define NXE2000_IRQSRC_ADC		(1 << 3)
#define NXE2000_IRQSRC_GPIO		(1 << 4)
#define NXE2000_IRQSRC_CHG		(1 << 6)
#define NXE2000_IRQSRC_FG		(1 << 7)
static irqreturn_t nxe2000_irq_thread(int irq, void *data)
{
	struct nxe2000_dev *nxe2000 = data;
	u8 *status_regs;
	u8	irq_src;
	int ret;
	int i;

	DBGOUT("%s\n", __func__);

	ret = nxe2000_read(nxe2000->i2c, NXE2000_REG_INTMON, &irq_src);
	if (ret < 0) {
		dev_err(nxe2000->dev, "Failed to read interrupt source: ret[%d], irq_src[0x%02x]\n",
				ret, irq_src);
		return IRQ_NONE;
	}

	status_regs = nxe2000->status_regs;
	memset(status_regs, 0x00, NXE2000_IRQ_GROUP_NR);

	if (irq_src & NXE2000_IRQSRC_SYSTEM) {
		status_regs[NXE2000_IRQGRP_SYSTEM_INT] = 0;
	}
	if (irq_src & NXE2000_IRQSRC_DCDC) {
		nxe2000_read(nxe2000->i2c, NXE2000_REG_DCIRQ,
				&status_regs[NXE2000_IRQGRP_DCDC_INT]);
	}
	if (irq_src & NXE2000_IRQSRC_ADC) {
		/* ADC INT1 ~ INT3 */
		nxe2000_bulk_reads(nxe2000->i2c, NXE2000_REG_IR_ADC1, 3,
				&status_regs[NXE2000_IRQGRP_ADC_INT1]);
	}
	if (irq_src & NXE2000_IRQSRC_GPIO) {
		u8 gpio_edge_info, gpio_intr_info[3];

		/* GPIO Rising & Falling Interrupt */
		nxe2000_read(nxe2000->i2c, NXE2000_REG_GPEDGE1,
				&gpio_edge_info);
		nxe2000_bulk_reads(nxe2000->i2c, NXE2000_REG_EN_GPIR, 3,
				gpio_intr_info);

		for (i = 0; i < NXE2000_NUM_GPIO; i++) {
			bool interrupt = false;

			switch (gpio_edge_info & (NXE2000_GPIO_INT_MASK << (i * 2))) {
			case NXE2000_GPIO_INT_BOTH:
				if (gpio_intr_info[0] & (gpio_intr_info[1] | gpio_intr_info[2]))
					interrupt = true;
				break;
			case NXE2000_GPIO_INT_RISE:
				if (gpio_intr_info[0] & gpio_intr_info[1])
					interrupt = true;
				break;
			case NXE2000_GPIO_INT_FALL:
				if (gpio_intr_info[0] & gpio_intr_info[2])
					interrupt = true;
				break;
			default:
				break;
			}

			if (interrupt) {
				status_regs[NXE2000_IRQGRP_GPIO_INT] |= (1 << i);
			}
		}
	}
	if (irq_src & NXE2000_IRQSRC_CHG) {
		nxe2000_bulk_reads(nxe2000->i2c, NXE2000_REG_CHGCTL_MONI, 4,
				&status_regs[NXE2000_IRQGRP_CHG_INT1]);
	}
	if (irq_src & NXE2000_IRQSRC_FG) {
		status_regs[NXE2000_IRQGRP_FG_INT] = 0;
	}

	/* Report */
	for (i = 0; i < NXE2000_IRQ_NR; i++) {
		if (status_regs[nxe2000_irqs[i].group] & nxe2000_irqs[i].mask) {
			handle_nested_irq(nxe2000->irq_base + i);
		}
	}

	/* Pending clear */
	for (i = 0; i < NXE2000_IRQ_GROUP_NR; i++)
	{
		if (nxe2000_mask_reg[i] == NXE2000_REG_TYPE_INVALID)
			continue;

		if (status_regs[i])
		{
#if 0
			if ((nxe2000_mask_reg[i] & 0xFF00) == NXE2000_REG_TYPE_INTEN)
			{
				nxe2000_update(nxe2000->i2c, (u8)(nxe2000_mask_reg[i] & 0xFF), ~status_regs[i], status_regs[i]);
			}
			else
			{
//				nxe2000_update(nxe2000->i2c, (u8)(nxe2000_mask_reg[i] & 0xFF), status_regs[i], status_regs[i]);
				nxe2000_write(nxe2000->i2c, (u8)(nxe2000_pend_reg[i] & 0xFF), status_regs[i]);
			}
#else

			nxe2000_update(nxe2000->i2c, (u8)(nxe2000_pend_reg[i] & 0xFF), ~status_regs[i], status_regs[i]);
#endif
		}
	}

	return IRQ_HANDLED;
}

int nxe2000_irq_resume(struct nxe2000_dev *nxe2000)
{
	DBGOUT("%s\n", __func__);

#if 0
	if (nxe2000->irq && nxe2000->irq_base)
		nxe2000_irq_thread(nxe2000->irq_base, nxe2000);
#endif
	return 0;
}

int nxe2000_irq_init(struct nxe2000_dev *nxe2000)
{
	int i;
	int cur_irq;
	int ret;

	DBGOUT("%s\n", __func__);

	if (!nxe2000->irq) {
		dev_warn(nxe2000->dev,
			 "No interrupt specified, no interrupts\n");
		nxe2000->irq_base = 0;
		return 0;
	}

	if (!nxe2000->irq_base) {
		dev_err(nxe2000->dev,
			"No interrupt base specified, no interrupts\n");
		return 0;
	}

	mutex_init(&nxe2000->irqlock);

	/* Disable top level interrupts */
	nxe2000_write(nxe2000->i2c, NXE2000_REG_INTEN,		0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_DCIREN,		0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_EN_ADCIR1,	0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_EN_ADCIR2,	0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_EN_ADCIR3,	0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_EN_GPIR,	0x00);

	/* Mask the individual interrupt sources */
	for (i = 0; i < NXE2000_IRQ_GROUP_NR; i++) {
		nxe2000->irq_masks_cur[i] = 0xff;
		nxe2000->irq_masks_cache[i] = 0xff;

		if (nxe2000_mask_reg[i] == NXE2000_REG_TYPE_INVALID)
			continue;

		nxe2000_write(nxe2000->i2c, (u8)(nxe2000_mask_reg[i] & 0xFF), 0xff);
	}

	/* Pending clear */
	nxe2000_write(nxe2000->i2c, NXE2000_REG_DCIRQ,		0x00);

	nxe2000_write(nxe2000->i2c, NXE2000_REG_IR_ADC1,	0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_IR_ADC2,	0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_IR_ADC3,	0x00);

	nxe2000_write(nxe2000->i2c, NXE2000_REG_IR_GPR,		0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_IR_GPF,		0x00);

	nxe2000_write(nxe2000->i2c, NXE2000_REG_CHGCTL_IRR,		0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_CHGSTAT_IRR1,	0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_CHGSTAT_IRR2,	0x00);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_CHGERR_IRR,		0x00);

	/* Register with genirq */
	for (cur_irq = nxe2000->irq_base;
		cur_irq < (NXE2000_IRQ_NR + nxe2000->irq_base);
		cur_irq++) {
		irq_set_chip_data(cur_irq, nxe2000);
		irq_set_chip_and_handler(cur_irq, &nxe2000_irq_chip,
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

#if 0
	if (nxe2000->irq) {
		ret = request_threaded_irq(gpio_to_irq(nxe2000->irq), NULL, nxe2000_irq_thread,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   "nxe2000-irq", nxe2000);
		if (ret) {
			dev_err(nxe2000->dev, "Failed to request IRQ %d: %d\n",
				nxe2000->irq, ret);
			return ret;
		}
	} else {
		dev_warn(nxe2000->dev,
			 "No interrupt specified - functionality limited\n");
	}

	/* Interrup enable */
//	nxe2000_write(nxe2000->i2c, NXE2000_REG_EN_ADCIR1,	0xff);
//	nxe2000_write(nxe2000->i2c, NXE2000_REG_EN_ADCIR2,	0xff);
//	nxe2000_write(nxe2000->i2c, NXE2000_REG_EN_ADCIR3,	0xff);
//	nxe2000_write(nxe2000->i2c, NXE2000_REG_EN_GPIR,	0xff);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_DCIREN, 	0xff);
	nxe2000_write(nxe2000->i2c, NXE2000_REG_INTEN,		0xdb);
#endif

	if (!nxe2000->ono)
		return 0;

	ret = request_threaded_irq(gpio_to_irq(nxe2000->ono), NULL, nxe2000_irq_thread,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING |
				   IRQF_ONESHOT, "nxe2000-ono", nxe2000);
	if (ret)
		dev_err(nxe2000->dev, "Failed to request ono-IRQ %d: %d\n",
			nxe2000->ono, ret);

	return 0;
}

void nxe2000_irq_exit(struct nxe2000_dev *nxe2000)
{
	DBGOUT("%s\n", __func__);

	if (nxe2000->ono)
		free_irq(nxe2000->ono, nxe2000);

	if (nxe2000->irq)
		free_irq(nxe2000->irq, nxe2000);
}
