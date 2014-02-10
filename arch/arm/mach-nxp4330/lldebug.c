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
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <linux/amba/serial.h>

#include <mach/platform.h>
#include <mach/soc.h>

/*
 * Macro
 */
#define	UART_DEBUG_HZ				CFG_UART_CLKGEN_CLOCK_HZ
#define	UART_DEBUG_BAUDRATE			CFG_UART_DEBUG_BAUDRATE
#define	LOCK_INTERRUPT				(1)

#if	  (0 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		IO_ADDRESS(PHY_BASEADDR_UART0)
	#define	UART_CLKG_BASE		IO_ADDRESS(PHY_BASEADDR_CLKGEN22)
	#define	RESET_UART_ID		RESET_ID_UART0
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART0_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART0_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART0_SMCRXENB
#elif (1 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		IO_ADDRESS(PHY_BASEADDR_UART1)
	#define	UART_CLKG_BASE		IO_ADDRESS(PHY_BASEADDR_CLKGEN24)
	#define	RESET_UART_ID		RESET_ID_UART1
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART_MODEM0_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART_MODEM0_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART_MODEM0_SMCRXENB
#elif (2 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		IO_ADDRESS(PHY_BASEADDR_UART2)
	#define	UART_CLKG_BASE		IO_ADDRESS(PHY_BASEADDR_CLKGEN23)
	#define	RESET_UART_ID		RESET_ID_UART2
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART1_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART1_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART1_SMCRXENB
#elif (3 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		IO_ADDRESS(PHY_BASEADDR_UART3)
	#define	UART_CLKG_BASE		IO_ADDRESS(PHY_BASEADDR_CLKGEN25)
	#define	RESET_UART_ID		RESET_ID_UART3
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART_NODMA0_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART_NODMA0_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART_NODMA0_SMCRXENB
#elif (4 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		IO_ADDRESS(PHY_BASEADDR_UART4)
	#define	UART_CLKG_BASE		IO_ADDRESS(PHY_BASEADDR_CLKGEN26)
	#define	RESET_UART_ID		RESET_ID_UART4
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART_NODMA1_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART_NODMA1_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART_NODMA1_SMCRXENB
#elif (5 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		IO_ADDRESS(PHY_BASEADDR_UART5)
	#define	UART_CLKG_BASE		IO_ADDRESS(PHY_BASEADDR_CLKGEN27)
	#define	RESET_UART_ID		RESET_ID_UART5
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART_NODMA2_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART_NODMA2_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART_NODMA2_SMCRXENB
#else
	#error not support low debug out uart port (0 ~ 5)
#endif


/*
 * Registers
 */
#define UART_PL01x_FR_TXFE              0x80
#define UART_PL01x_FR_RXFF              0x40
#define UART_PL01x_FR_TXFF              0x20
#define UART_PL01x_FR_RXFE              0x10
#define UART_PL01x_FR_BUSY              0x08
#define UART_PL01x_FR_TMSK              (UART_PL01x_FR_TXFF + UART_PL01x_FR_BUSY)

#define UART_PL011_LCRH_SPS             (1 << 7)
#define UART_PL011_LCRH_WLEN_8          (3 << 5)
#define UART_PL011_LCRH_WLEN_7          (2 << 5)
#define UART_PL011_LCRH_WLEN_6          (1 << 5)
#define UART_PL011_LCRH_WLEN_5          (0 << 5)
#define UART_PL011_LCRH_FEN             (1 << 4)
#define UART_PL011_LCRH_STP2            (1 << 3)
#define UART_PL011_LCRH_EPS             (1 << 2)
#define UART_PL011_LCRH_PEN             (1 << 1)
#define UART_PL011_LCRH_BRK             (1 << 0)

#define UART_PL011_CR_CTSEN      	(1 << 15)
#define UART_PL011_CR_RTSEN         (1 << 14)
#define UART_PL011_CR_OUT2          (1 << 13)
#define UART_PL011_CR_OUT1          (1 << 12)
#define UART_PL011_CR_RTS           (1 << 11)
#define UART_PL011_CR_DTR           (1 << 10)
#define UART_PL011_CR_RXE           (1 << 9)
#define UART_PL011_CR_TXE           (1 << 8)
#define UART_PL011_CR_LPE           (1 << 7)
#define UART_PL011_CR_IIRLP         (1 << 2)
#define UART_PL011_CR_SIREN         (1 << 1)
#define UART_PL011_CR_UARTEN        (1 << 0)

struct pl01x_regs {
    u32 dr;    		/* 0x00 Data register */
    u32 ecr;        /* 0x04 Error clear register (Write) */
    u32 pl010_lcrh; /* 0x08 Line control register, high byte */
    u32 pl010_lcrm; /* 0x0C Line control register, middle byte */
    u32 pl010_lcrl; /* 0x10 Line control register, low byte */
    u32 pl010_cr;   /* 0x14 Control register */
    u32 fr;     	/* 0x18 Flag register (Read only) */
#ifdef CONFIG_PL011_SERIAL_RLCR
    u32 pl011_rlcr; /* 0x1c Receive line control register */
#else
    u32 reserved;
#endif
    u32 ilpr;       /* 0x20 IrDA low-power counter register */
    u32 pl011_ibrd; /* 0x24 Integer baud rate register */
    u32 pl011_fbrd; /* 0x28 Fractional baud rate register */
    u32 pl011_lcrh; /* 0x2C Line control register */
    u32 pl011_cr;   /* 0x30 Control register */
};

struct uart_data {
	/* clkgen */
	int pll;
	int div;
	long rate;
	/* uart */
	unsigned int divider;
	unsigned int fraction;
	unsigned int lcr;
	unsigned int cr;
};

/*
 * Low level debug function.
 * default debug port is '0'
 */
static struct uart_data clk = { 0 ,};

#define	MAX_DIVIDER			((1<<8) - 1)	// 256, align 2
#define	DIVIDER_ALIGN		2

static long calc_uart_clock(long request, int *pllsel, int *plldiv)
{
	struct clk *clk;
	unsigned long rate = 0, clkhz[3], freqhz = 0, pllhz;
	int pll = 0, div = 0, divide, maxdiv, align, n;

	clk = clk_get(NULL, "pll0"), clkhz[0] = clk_get_rate(clk), clk_put(clk);
	clk = clk_get(NULL, "pll1"), clkhz[1] = clk_get_rate(clk), clk_put(clk);
	clk = clk_get(NULL, "pll2"), clkhz[2] = clk_get_rate(clk), clk_put(clk);

	for (n = 0; ARRAY_SIZE(clkhz) > n; n++) {
	#ifdef  CONFIG_ARM_NXP4330_CPUFREQ
		if (n == CONFIG_NXP4330_CPUFREQ_PLLDEV)
			continue;
	#endif
		pllhz = clkhz[n];
		divide = (pllhz/request);
		maxdiv = MAX_DIVIDER & ~(DIVIDER_ALIGN-1);
		align = (divide & ~(DIVIDER_ALIGN-1)) + DIVIDER_ALIGN;

		if (!divide) {
			divide = 1;
		} else {
			if (1 != divide)
				divide &= ~(DIVIDER_ALIGN-1);

			if (divide != align &&
				abs(request - pllhz/divide) >
				abs(request - pllhz/align))
				divide = align;

			divide = (divide > maxdiv ? maxdiv : divide);
		}
		freqhz = pllhz / divide;

		if (rate && (abs(freqhz-request) > abs(rate-request)))
			continue;

		rate = freqhz;
		div = divide;
		pll = n;
	}

	if (pllsel)
		*pllsel = pll;

	if (plldiv)
		*plldiv = div;

	return rate;
}

inline static void uart_init(void)
{
	U32 CLKENB = UART_CLKG_BASE;
	U32 CLKGEN = UART_CLKG_BASE + 0x04;
	struct uart_data *pdat = &clk;
	struct pl01x_regs *regs = (struct pl01x_regs *)UART_PHYS_BASE;
	unsigned int baudrate = UART_DEBUG_BAUDRATE;
	unsigned int temp;

	/*
	 * Clock Generotor & reset
	 */
	if (0 == pdat->rate) {
		unsigned int remainder;

		/*
		 * Set baud rate
	 	 *
	 	 * IBRD = UART_CLK / (16 * BAUD_RATE)
	 	 * FBRD = RND((64 * MOD(UART_CLK,(16 * BAUD_RATE))) / (16 * BAUD_RATE))
 	 	 */
		pdat->rate = calc_uart_clock(UART_DEBUG_HZ, &pdat->pll, &pdat->div);
		temp = 16 * baudrate;
		pdat->divider = pdat->rate / temp;
		remainder = pdat->rate % temp;
		temp = (8 * remainder) / baudrate;
		pdat->fraction = (temp >> 1) + (temp & 1);
		pdat->lcr = UART_PL011_LCRH_WLEN_8 | UART_PL011_LCRH_FEN;
		pdat->cr = UART_PL011_CR_UARTEN | UART_PL011_CR_TXE | UART_PL011_CR_RXE |
	   				UART_PL011_CR_RTS;
	}

	/* check reset */
	if (!nxp_soc_rsc_status(RESET_UART_ID)) {
		NX_TIEOFF_Set(TIEOFF_USESMC, 0);
		NX_TIEOFF_Set(TIEOFF_SMCTXENB, 0);
		NX_TIEOFF_Set(TIEOFF_SMCRXENB, 0);
		nxp_soc_rsc_reset(RESET_UART_ID);
	}

	/* check pll */
	if (!(pdat->pll & (readl(CLKGEN)>>2 & 0x7))) {
		writel(readl(CLKENB) & ~(1<<2), CLKENB);
		temp = readl(CLKGEN) & ~(0x07<<2) & ~(0xFF<<5);
		writel((temp|(pdat->pll<<2)|((pdat->div-1)<<5)), CLKGEN);
	}

	/*
	 * Uart
	 */
	writel(0, &regs->pl011_cr);	// First, disable everything
	writel(pdat->divider, &regs->pl011_ibrd);
	writel(pdat->fraction, &regs->pl011_fbrd);
	writel(pdat->lcr, &regs->pl011_lcrh);	// Set the UART to be 8 bits, 1 stop bit, no parity, fifo enabled
	writel(pdat->cr, &regs->pl011_cr);	// Finally, enable the UART

	/*
	 * alaway enable clkgen
	 */
	writel((readl(CLKENB)|(1<<2)), CLKENB);
}

inline static void uart_putc(char ch)
{
	struct pl01x_regs *regs = (struct pl01x_regs *)UART_PHYS_BASE;
	unsigned int status;

	/* Wait until there is space in the FIFO */
	while (readl(&regs->fr) & UART_PL01x_FR_TXFF)
	{ ; }

	/* Send the character */
	writel(ch, &regs->dr);

	/*
     *  Finally, wait for transmitter to become empty
 	 */
    do {
        status = readw(&regs->fr);
    } while (status & UART_PL01x_FR_BUSY);
}

inline static char uart_getc(void)
{
	struct pl01x_regs *regs = (struct pl01x_regs *)UART_PHYS_BASE;
	unsigned int data;

	/* Wait until there is data in the FIFO */
	while (readl(&regs->fr) & UART_PL01x_FR_RXFE)
	{ ; }

	data = readl(&regs->dr);

	/* Check for an error flag */
	if (data & 0xFFFFFF00) {
		/* Clear the error */
		writel(0xFFFFFFFF, &regs->ecr);
		return -1;
	}

	return (int) data;
}

inline static int uart_tstc(void)
{
	struct pl01x_regs *regs = (struct pl01x_regs *)UART_PHYS_BASE;
	return !(readl(&regs->fr) & UART_PL01x_FR_RXFE);
}

/*
 * Low level uart interface
 */
void lldebug_init(void)
{
	uart_init();
}

void lldebug_putc(const char ch)
{
   /* If \n, also do \r */
	if (ch == '\n')
    	uart_putc('\r');
	uart_putc(ch);
}

int lldebug_getc(void)
{
	return uart_getc();
}

void lldebug_puts(const char *str)
{
	while (*str)
		lldebug_putc(*str++);
}

int lldebug_tstc(void)
{
	return uart_tstc();
}

/*
 * Low level debug interface.
 */
void lldebugout(const char *fmt, ...)
{
	va_list va;
	char buff[256];
	u_long flags;

#if	LOCK_INTERRUPT
	/* disable irq */
	local_irq_save(flags);
#endif

	lldebug_init();

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	/* direct debug out */
	lldebug_puts(buff);

#if	LOCK_INTERRUPT
	/* enable irq */
	local_irq_restore(flags);
#endif
}
EXPORT_SYMBOL_GPL(lldebugout);
