/*
 * COM1 NS16550 support
 * originally from linux source (arch/powerpc/boot/ns16550.c)
 * modified to use CONFIG_SYS_ISA_MEM and new defines
 */

#include <config.h>
#include <ns16550.h>
#include <watchdog.h>
#include <linux/types.h>
#include <asm/io.h>

#define UART_LCRVAL UART_LCR_8N1		/* 8 data, 1 stop, no parity */
#define UART_MCRVAL (UART_MCR_DTR | \
		     UART_MCR_RTS)		/* RTS/DTR */
#define UART_FCRVAL (UART_FCR_FIFO_EN |	\
		     UART_FCR_RXSR |	\
		     UART_FCR_TXSR)		/* Clear & enable FIFOs */
#ifdef CONFIG_SYS_NS16550_PORT_MAPPED
#define serial_out(x, y)	outb(x, (ulong)y)
#define serial_in(y)		inb((ulong)y)
#elif defined(CONFIG_SYS_NS16550_MEM32) && (CONFIG_SYS_NS16550_REG_SIZE > 0)
#define serial_out(x, y)	out_be32(y, x)
#define serial_in(y)		in_be32(y)
#elif defined(CONFIG_SYS_NS16550_MEM32) && (CONFIG_SYS_NS16550_REG_SIZE < 0)
#define serial_out(x, y)	out_le32(y, x)
#define serial_in(y)		in_le32(y)
#else
#define serial_out(x, y)	writeb(x, y)
#define serial_in(y)		readb(y)
#endif

#ifndef CONFIG_SYS_NS16550_IER
#define CONFIG_SYS_NS16550_IER  0x00
#endif /* CONFIG_SYS_NS16550_IER */

#if (defined(CONFIG_OMAP) && !defined(CONFIG_OMAP3_ZOOM2)) || \
					defined(CONFIG_AM33XX)
static inline void _sdelay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}

/**
 * Workaround for Errata i202 (3430 - 1.12, 3630 - 1.6, 4430/4460 - 1.1)
 *
 * MDR1 access can freeze UART module
 *
 * DESCRIPTION:
 *
 * Because of a glitchy structure inside the UART module, accessing the MDR1 register may create a
 * dummy underrun condition and freeze the UART in IrDa transmission. In UART mode, this may corrupt
 * the transferred data(received or transmitted).
 *
 * WORKAROUND:
 *
 * To ensure this problem does not occur, the following software initialization sequence must be used each
 * time MDR1 must be changed:
 *
 * 1. If needed, setup the UART by writing the required registers, except MDR1
 * 2. Set appropriately the MDR1.MODE_SELECT bit field
 * 3. Wait for 5 L4 clock cycles + 5 UART functional clock cycles
 * 4. Clear TX and RX FIFO in FCR register to reset its counter logic
 * 5. Read RESUME register to resume the halted operation
 */
void omap_uart_mdr1_errata_i202(NS16550_t com_port,
		unsigned char mdr1_val, unsigned char fcr_val)
{
#define UART_LSR_DATA_INOUT (UART_LSR_DR | UART_LSR_TEMT | UART_LSR_THRE)
#define UART_LSR_DATA_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)
	/* 10 retries, in this the FiFO's should get cleared */
	unsigned char timeout = 10;

	serial_out(mdr1_val, &com_port->mdr1);
	_sdelay(2000); /* 1 us: Enough loops assuming a maximum of 2GHz */
	serial_out(fcr_val, &com_port->fcr);

	/* Wait for FIFO to empty: when empty, RX bit is 0 and TX bits is 1. */
	while ((serial_in(&com_port->lsr) & UART_LSR_DATA_INOUT) != UART_LSR_DATA_EMPTY) {
		if (!(--timeout)) {
			break;
		}
		_sdelay(1000);
	}
}
#endif /* CONFIG_OMAP */

void NS16550_init(NS16550_t com_port, int baud_divisor)
{
	serial_out(CONFIG_SYS_NS16550_IER, &com_port->ier);
#if (defined(CONFIG_OMAP) && !defined(CONFIG_OMAP3_ZOOM2)) || \
					defined(CONFIG_AM33XX)
	serial_out(0x7, &com_port->mdr1);	/* mode select reset TL16C750*/
#endif
	serial_out(UART_LCR_BKSE | UART_LCRVAL, (ulong)&com_port->lcr);
	serial_out(0, &com_port->dll);
	serial_out(0, &com_port->dlm);
	serial_out(UART_LCRVAL, &com_port->lcr);
	serial_out(UART_MCRVAL, &com_port->mcr);
	serial_out(UART_FCRVAL, &com_port->fcr);
	serial_out(UART_LCR_BKSE | UART_LCRVAL, &com_port->lcr);
	serial_out(baud_divisor & 0xff, &com_port->dll);
	serial_out((baud_divisor >> 8) & 0xff, &com_port->dlm);
	serial_out(UART_LCRVAL, &com_port->lcr);
#if (defined(CONFIG_OMAP) && !defined(CONFIG_OMAP3_ZOOM2)) || \
	defined(CONFIG_AM33XX) || defined(CONFIG_SOC_DA8XX)

#if 0
#if defined(CONFIG_APTIX)
	/* /13 mode so Aptix 6MHz can hit 115200 */
	serial_out(3, &com_port->mdr1);
#else
	/* /16 is proper to hit 115200 with 48MHz */
	serial_out(0, &com_port->mdr1);
#endif
#else
#if defined(CONFIG_APTIX)
	/* /13 mode so Aptix 6MHz can hit 115200 */
	omap_uart_mdr1_errata_i202(com_port, 3, UART_FCRVAL);
#else
	/* /16 is proper to hit 115200 with 48MHz */
	omap_uart_mdr1_errata_i202(com_port, 0, UART_FCRVAL);
#endif
#endif
#endif /* CONFIG_OMAP */
}

#ifndef CONFIG_NS16550_MIN_FUNCTIONS
void NS16550_reinit(NS16550_t com_port, int baud_divisor)
{
	serial_out(CONFIG_SYS_NS16550_IER, &com_port->ier);
	serial_out(UART_LCR_BKSE | UART_LCRVAL, &com_port->lcr);
	serial_out(0, &com_port->dll);
	serial_out(0, &com_port->dlm);
	serial_out(UART_LCRVAL, &com_port->lcr);
	serial_out(UART_MCRVAL, &com_port->mcr);
	serial_out(UART_FCRVAL, &com_port->fcr);
	serial_out(UART_LCR_BKSE, &com_port->lcr);
	serial_out(baud_divisor & 0xff, &com_port->dll);
	serial_out((baud_divisor >> 8) & 0xff, &com_port->dlm);
	serial_out(UART_LCRVAL, &com_port->lcr);
}
#endif /* CONFIG_NS16550_MIN_FUNCTIONS */

void NS16550_putc(NS16550_t com_port, char c)
{
	while ((serial_in(&com_port->lsr) & UART_LSR_THRE) == 0)
		;
	serial_out(c, &com_port->thr);

	/*
	 * Call watchdog_reset() upon newline. This is done here in putc
	 * since the environment code uses a single puts() to print the complete
	 * environment upon "printenv". So we can't put this watchdog call
	 * in puts().
	 */
	if (c == '\n')
		WATCHDOG_RESET();
}

#ifndef CONFIG_NS16550_MIN_FUNCTIONS
char NS16550_getc(NS16550_t com_port)
{
	while ((serial_in(&com_port->lsr) & UART_LSR_DR) == 0) {
#ifdef CONFIG_USB_TTY
		extern void usbtty_poll(void);
		usbtty_poll();
#endif
		WATCHDOG_RESET();
	}
	return serial_in(&com_port->rbr);
}

int NS16550_tstc(NS16550_t com_port)
{
	return (serial_in(&com_port->lsr) & UART_LSR_DR) != 0;
}

#endif /* CONFIG_NS16550_MIN_FUNCTIONS */
