/*
 * drivers/serial/omap-serial.c
 *
 * Driver for OMAP3430 UART controller.
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Authors:
 *	Govindraj R	<govindraj.raja@ti.com>
 *	Thara Gopinath	<thara@ti.com>
 *
 * PM Framework based on arch/arm/mach-omap2/serial.c
 * by Kevin Hilman.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/serial_reg.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <asm/irq.h>
#include <plat/dma.h>
#include <plat/dmtimer.h>
#include <mach/omap-serial.h>

static struct uart_omap_port *ui[OMAP_MAX_HSUART_PORTS];
struct omap_hsuart_state omap_hsuart[OMAP_MAX_HSUART_PORTS];

/* Forward declaration of functions */
static void uart_tx_dma_callback(int lch, u16 ch_status, void *data);
static void serial_omap_rx_timeout(unsigned long uart_no);
static void serial_omap_start_rxdma(struct uart_omap_port *up);
static int serial_omap_remove(struct platform_device *dev);

static inline void omap_uart_block_sleep(int num);
static void omap_uart_smart_idle_enable(int num, int enable);
static inline void omap_uart_enable_clocks(int num);

#ifdef DEBUG
static void serial_omap_display_reg(struct uart_port *port);
#endif

int console_detect(char *str)
{
	char *next, *start = NULL;
	int i;

	i = strlen(CONSOLE_NAME);
	next = saved_command_line;

	while ((next = strchr(next, 'c')) != NULL) {
		if (!strncmp(next, CONSOLE_NAME, i)) {
			start = next;
			break;
		} else {
			next++;
		}
	}
	if (!start)
		return -EPERM;
	i = 0;
	start = strchr(start, '=') + 1;
	while (*start != ',') {
		str[i++] = *start++;
		if (i > 6) {
			pr_err("%s : Invalid Console Name\n", __func__);
			return -EPERM;
		}
	}
	str[i] = '\0';
	return 0;
}

static inline unsigned int serial_in(struct uart_omap_port *up, int offset)
{
	offset <<= up->port.regshift;
	return readw(up->port.membase + offset);
}

static inline void serial_out(struct uart_omap_port *up, int offset, int value)
{
	offset <<= up->port.regshift;
	writew(value, up->port.membase + offset);
}

static inline void serial_omap_clear_fifos(struct uart_omap_port *port)
{
		struct uart_omap_port *up = (struct uart_omap_port *)port;
		serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
			       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_out(up, UART_FCR, 0);
}

/*
 * We have written our own function to get the divisor so as to support
 * 13x mode.
 */
static unsigned int
serial_omap_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int divisor;
	if (baud > OMAP_MODE13X_SPEED && baud != 3000000)
		divisor = 13;
	else
		divisor = 16;
	return port->uartclk/(baud * divisor);
}

static void serial_omap_stop_rxdma(struct uart_omap_port *up)
{
	if (up->uart_dma.rx_dma_state) {
		del_timer_sync(&up->uart_dma.rx_timer);
		omap_stop_dma(up->uart_dma.rx_dma_channel);
		omap_free_dma(up->uart_dma.rx_dma_channel);
		up->uart_dma.rx_dma_channel = 0xFF;
		up->uart_dma.rx_dma_state = 0x0;
	}
}

static void serial_omap_enable_ms(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	dev_dbg(up->port.dev, "serial_omap_enable_ms+%d\n", up->pdev->id);
	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}

static void serial_omap_stop_tx(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	if (up->use_dma) {
		if (up->uart_dma.tx_dma_channel != 0xFF) {
			/*
			 * Check if dma is still active . If yes do nothing,
			 * return. Else stop dma
			 */
			int status = omap_readl(OMAP34XX_DMA4_BASE +
				    OMAP_DMA4_CCR(up->uart_dma.tx_dma_channel));
			if (status & (1 << 7))
				return;
			omap_stop_dma(up->uart_dma.tx_dma_channel);
			omap_free_dma(up->uart_dma.tx_dma_channel);
			up->uart_dma.tx_dma_channel = 0xFF;
		}
	}

	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

static void serial_omap_stop_rx(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	serial_omap_stop_rxdma(up);
	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
	/*Disable the UART CTS wakeup for UART1,UART2*/
	if ((!port->suspended && (((up->pdev->id - 1) == UART1) ||
			((up->pdev->id  - 1) == UART2))))
		omap_uart_cts_wakeup((up->pdev->id - 1), 0);
}

static inline void receive_chars(struct uart_omap_port *up, int *status)
{
	struct tty_struct *tty = up->port.info->port.tty;
	unsigned int ch, flag;
	int max_count = 256;

	do {
		ch = serial_in(up, UART_RX);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(*status & (UART_LSR_BI | UART_LSR_PE |
				       UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (*status & UART_LSR_PE)
				up->port.icount.parity++;
			else if (*status & UART_LSR_FE)
				up->port.icount.frame++;
			if (*status & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			*status &= up->port.read_status_mask;

#ifdef CONFIG_SERIAL_OMAP_CONSOLE
			if (up->port.line == up->port.cons->index) {
				/* Recover the break flag from console xmit */
				*status |= up->lsr_break_flag;
				up->lsr_break_flag = 0;
			}
#endif
			if (*status & UART_LSR_BI)
				flag = TTY_BREAK;
			else if (*status & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (*status & UART_LSR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;
		uart_insert_char(&up->port, *status, UART_LSR_OE, ch, flag);
ignore_char:
		*status = serial_in(up, UART_LSR);
	} while ((*status & UART_LSR_DR) && (max_count-- > 0));
	tty_flip_buffer_push(tty);
}

static void transmit_chars(struct uart_omap_port *up)
{
	struct circ_buf *xmit = &up->port.info->xmit;
	int count;

	if (up->port.x_char) {
		serial_out(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		serial_omap_stop_tx(&up->port);
		return;
	}

	count = up->port.fifosize / 4;
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		serial_omap_stop_tx(&up->port);
}

static void serial_omap_start_tx(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	if (up->use_dma && !(up->port.x_char)) {

		struct circ_buf *xmit = &up->port.info->xmit;
		unsigned int start = up->uart_dma.tx_buf_dma_phys +
				     (xmit->tail & (UART_XMIT_SIZE - 1));
		if (uart_circ_empty(xmit) || up->uart_dma.tx_dma_state)
			return;
		spin_lock(&(up->uart_dma.tx_lock));
		up->uart_dma.tx_dma_state = 1;
		spin_unlock(&(up->uart_dma.tx_lock));

		up->uart_dma.tx_buf_size = uart_circ_chars_pending(xmit);
		/* It is a circular buffer. See if the buffer has wounded back.
		 * If yes it will have to be transferred in two separate dma
		 * transfers */
		if (start + up->uart_dma.tx_buf_size >=
				up->uart_dma.tx_buf_dma_phys + UART_XMIT_SIZE)
			up->uart_dma.tx_buf_size =
				(up->uart_dma.tx_buf_dma_phys +
				UART_XMIT_SIZE) - start;

		if (up->uart_dma.tx_dma_channel == 0xFF)
			omap_request_dma(up->uart_dma.uart_dma_tx,
					"UART Tx DMA",
					(void *)uart_tx_dma_callback, up,
					&(up->uart_dma.tx_dma_channel));
		omap_set_dma_dest_params(up->uart_dma.tx_dma_channel, 0,
					 OMAP_DMA_AMODE_CONSTANT,
					 up->uart_dma.uart_base, 0, 0);
		omap_set_dma_src_params(up->uart_dma.tx_dma_channel, 0,
					OMAP_DMA_AMODE_POST_INC, start, 0, 0);

		omap_set_dma_transfer_params(up->uart_dma.tx_dma_channel,
					     OMAP_DMA_DATA_TYPE_S8,
					     up->uart_dma.tx_buf_size, 1,
					     OMAP_DMA_SYNC_ELEMENT,
					     up->uart_dma.uart_dma_tx, 0);

		omap_start_dma(up->uart_dma.tx_dma_channel);

	} else if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

static unsigned int check_modem_status(struct uart_omap_port *up)
{
	int status;
	status = serial_in(up, UART_MSR);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return status;
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.info != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change
				(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change
				(&up->port, status & UART_MSR_CTS);
		wake_up_interruptible(&up->port.info->delta_msr_wait);
	}

	return status;
}

/*
 * This handles the interrupt from one port.
 */
static inline irqreturn_t serial_omap_irq(int irq, void *dev_id)
{
	struct uart_omap_port *up = dev_id;
	unsigned int iir, lsr;

	omap_uart_block_sleep(up->pdev->id - 1);

	iir = serial_in(up, UART_IIR);
	if (iir & UART_IIR_NO_INT)
		return IRQ_NONE;

	lsr = serial_in(up, UART_LSR);
	if ((iir & 0x4) && up->use_dma) {
		up->ier &= ~UART_IER_RDI;
		serial_out(up, UART_IER, up->ier);
		serial_omap_start_rxdma(up);
	} else if (lsr & UART_LSR_DR) {
		receive_chars(up, &lsr);
	}
	check_modem_status(up);
	if ((lsr & UART_LSR_THRE) && (iir & 0x2))
		transmit_chars(up);

	up->port_activity = jiffies;
	return IRQ_HANDLED;
}

static unsigned int serial_omap_tx_empty(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags;
	unsigned int ret;

	dev_dbg(up->port.dev, "serial_omap_tx_empty+%d\n", up->pdev->id);
	spin_lock_irqsave(&up->port.lock, flags);
	ret = serial_in(up, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return ret;
}

static unsigned int serial_omap_get_mctrl(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char status;
	unsigned int ret;

	status = check_modem_status(up);
	dev_dbg(up->port.dev, "serial_omap_get_mctrl+%d\n", up->pdev->id);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial_omap_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char mcr = 0;

	dev_dbg(up->port.dev, "serial_omap_set_mctrl+%d\n", up->pdev->id);
	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr |= up->mcr;
	serial_out(up, UART_MCR, mcr);
}

static void serial_omap_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags;

	dev_dbg(up->port.dev, "serial_omap_break_ctl+%d\n", up->pdev->id);
	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int serial_omap_startup(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags;
	int irq_flags = 0;
	int retval;

	/*Enable the UART CTS wakeup for UART1,UART2*/
	if (((up->pdev->id - 1) == UART1) || ((up->pdev->id  - 1) == UART2))
		omap_uart_cts_wakeup((up->pdev->id - 1), 1);
	omap_uart_smart_idle_enable(up->pdev->id - 1, 0);

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(up->port.irq, serial_omap_irq, irq_flags,
				up->name, up);
	if (retval)
		return retval;

	dev_dbg(up->port.dev, "serial_omap_startup+%d\n", up->pdev->id);

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial_omap_clear_fifos(up);
	/* For Hardware flow control */
	serial_out(up, UART_MCR, 0x2);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);
	spin_lock_irqsave(&up->port.lock, flags);
	if (up->port.flags & UPF_FOURPORT) {
		if (!is_real_interrupt(up->port.irq))
			up->port.mctrl |= TIOCM_OUT1;
	} else {
		/*
		 * Most PC uarts need OUT2 raised to enable interrupts.
		 */
		if (is_real_interrupt(up->port.irq))
			up->port.mctrl |= TIOCM_OUT2;
	}
	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	up->msr_saved_flags = 0;

	if (up->use_dma) {
		if (!up->is_buf_dma_alloced) {
			free_page((unsigned long)up->port.info->xmit.buf);
			up->port.info->xmit.buf = NULL;
			up->port.info->xmit.buf = dma_alloc_coherent(NULL,
				UART_XMIT_SIZE,
				(dma_addr_t *)&(up->uart_dma.tx_buf_dma_phys),
				0);
			up->is_buf_dma_alloced = 1;
		}
		init_timer(&(up->uart_dma.rx_timer));
		up->uart_dma.rx_timer.function = serial_omap_rx_timeout;
		up->uart_dma.rx_timer.data = up->pdev->id;
		/* Currently the buffer size is 4KB. Can increase it */
		up->uart_dma.rx_buf = dma_alloc_coherent(NULL,
			up->uart_dma.rx_buf_size,
			(dma_addr_t *)&(up->uart_dma.rx_buf_dma_phys), 0);
	}

	/*
	* Finally, enable interrupts.  Note: Modem status interrupts
	* are set via set_termios(), which will be occurring imminently
	* anyway, so we don't enable them here.
	*/
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_out(up, UART_IER, up->ier);

	up->port_activity = jiffies;
	return 0;
}

static void serial_omap_shutdown(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags;

	dev_dbg(up->port.dev, "serial_omap_shutdown+%d\n", up->pdev->id);
	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	if (up->port.flags & UPF_FOURPORT) {
		/* reset interrupts on the AST Fourport board */
		inb((up->port.iobase & 0xfe0) | 0x1f);
		up->port.mctrl |= TIOCM_OUT1;
	} else
		up->port.mctrl &= ~TIOCM_OUT2;
	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serial_omap_clear_fifos(up);

	/*
	 * Read data port to reset things, and then free the irq
	 */
	(void) serial_in(up, UART_RX);
	if (up->use_dma) {
		int tmp;
		if (up->is_buf_dma_alloced) {
			dma_free_coherent(up->port.dev,
				  UART_XMIT_SIZE,
				  up->port.info->xmit.buf,
				  up->uart_dma.tx_buf_dma_phys);
			up->port.info->xmit.buf = NULL;
			up->is_buf_dma_alloced = 0;
		}
		serial_omap_stop_rx(port);
		dma_free_coherent(up->port.dev,
				  up->uart_dma.rx_buf_size,
				  up->uart_dma.rx_buf,
				  up->uart_dma.rx_buf_dma_phys);
		up->uart_dma.rx_buf = NULL;
		tmp = serial_in(up, UART_OMAP_SYSC) & 0x7;
		serial_out(up, UART_OMAP_SYSC, tmp); /* force-idle */
	}
	free_irq(up->port.irq, up);
}

static inline void
serial_omap_configure_xonxoff
		(struct uart_omap_port *up, struct ktermios *termios)
{
	unsigned char efr = 0;

	up->lcr = serial_in(up, UART_LCR);
	serial_out(up, UART_LCR, 0xbf);
	up->efr = serial_in(up, UART_EFR);
	serial_out(up, UART_EFR, up->efr & ~UART_EFR_ECB);

	serial_out(up, UART_XON1, termios->c_cc[VSTART]);
	serial_out(up, UART_XOFF1, termios->c_cc[VSTOP]);

	/* IXON Flag:
	 * Enable XON/XOFF flow control on output.
	 * Transmit XON1, XOFF1
	 */
	if (termios->c_iflag & IXON)
		efr |= 0x01 << 3;

	/* IXOFF Flag:
	 * Enable XON/XOFF flow control on input.
	 * Receiver compares XON1, XOFF1.
	 */
	if (termios->c_iflag & IXOFF)
		efr |= 0x01 << 1;

	serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);
	serial_out(up, UART_LCR, 0x80);

	up->mcr = serial_in(up, UART_MCR);

	/* IXANY Flag:
	 * Enable any character to restart output.
	 * Operation resumes after receiving any
	 * character after recognition of the XOFF character
	 */
	if (termios->c_iflag & IXANY)
		up->mcr |= 1<<5;

	serial_out(up, UART_MCR, up->mcr | 1<<6);

	serial_out(up, UART_LCR, 0xbf);
	serial_out(up, UART_TI752_TCR, 0x0f);
	/* Enable special char function UARTi.EFR_REG[5] and
	 * load the new software flow control mode IXON or IXOFF
	 * and restore the UARTi.EFR_REG[4] ENHANCED_EN value.
	 */
	serial_out(up, UART_EFR, up->efr | efr | 1<<5);
	serial_out(up, UART_LCR, 0x80);

	serial_out(up, UART_MCR, up->mcr & ~(1<<6));
	serial_out(up, UART_LCR, up->lcr);
}

static void
serial_omap_set_termios(struct uart_port *port, struct ktermios *termios,
			struct ktermios *old)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char cval;
	unsigned char efr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;

	/*
	 * Ask the core to calculate the divisor for us.
	 */

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/13);
	quot = serial_omap_get_divisor(port, baud);

	if (up->use_dma)
		up->fcr = UART_FCR_ENABLE_FIFO
					| 0x1 << 6 | 0x1 << 4
					| UART_FCR_DMA_SELECT;
	else
		up->fcr = UART_FCR_ENABLE_FIFO
					| 0x1 << 6 | 0x1 << 4;

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characters to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * Modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
	serial_out(up, UART_LCR, cval);		/* reset DLAB */

	/*-----------FIFOs and DMA Settings -----------*/
	serial_out(up, UART_LCR, UART_LCR_DLAB);
	serial_out(up, UART_DLL, 0);
	serial_out(up, UART_DLM, 0);
	serial_out(up, UART_LCR, 0);

	serial_out(up, UART_LCR, 0xbf);

	up->efr = serial_in(up, UART_EFR);
	serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);
	serial_out(up, UART_LCR, 0x0);			/* Access FCR */

	up->mcr = serial_in(up, UART_MCR);
	serial_out(up, UART_MCR, up->mcr | 0x40);        /* Access TLR*/
	/* FIFO ENABLE, DMA MODE */
	serial_out(up, UART_FCR, up->fcr);
	serial_out(up, UART_LCR, 0xbf);			/* Access EFR */

	if (up->use_dma) {
		serial_out(up, UART_TI752_TLR, 0x00);
		serial_out(up, UART_OMAP_SCR, ((1 << 6) | (1 << 7)));
	}

	serial_out(up, UART_EFR, up->efr);
	serial_out(up, UART_LCR, 0x80);
	serial_out(up, UART_MCR, up->mcr);        /* Restore TLR */

	/*-----Protocol, Baud Rate, and Interrupt Settings -- */

	serial_out(up, UART_OMAP_MDR1, OMAP_MDR1_DISABLE);

	serial_out(up, UART_LCR, 0xbf); /* Access EFR */

	up->efr = serial_in(up, UART_EFR);
	serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);

	serial_out(up, UART_LCR, 0);
	serial_out(up, UART_IER, 0);
	serial_out(up, UART_LCR, 0xbf);

	serial_out(up, UART_DLL, quot & 0xff);          /* LS of divisor */
	serial_out(up, UART_DLM, quot >> 8);            /* MS of divisor */

	serial_out(up, UART_LCR, 0);
	serial_out(up, UART_IER, up->ier);
	serial_out(up, UART_LCR, 0xbf); /* Access EFR */

	serial_out(up, UART_EFR, up->efr);
	serial_out(up, UART_LCR, cval);

	if (baud > 230400 && baud != 3000000)
		serial_out(up, UART_OMAP_MDR1, OMAP_MDR1_MODE13X);
	else
		serial_out(up, UART_OMAP_MDR1, OMAP_MDR1_MODE16X);

	/* Hardware Flow Control Configuration */

	if (termios->c_cflag & CRTSCTS) {
		efr |= (UART_EFR_CTS | UART_EFR_RTS);
		serial_out(up, UART_LCR, 0x80);

		up->mcr = serial_in(up, UART_MCR);
		serial_out(up, UART_MCR, up->mcr | 0x40);

		serial_out(up, UART_LCR, 0xbf); /* Access EFR */

		up->efr = serial_in(up, UART_EFR);
		serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);

		serial_out(up, UART_TI752_TCR, 0x0f);
		serial_out(up, UART_EFR, efr); /* Enable AUTORTS and AUTOCTS */
		serial_out(up, UART_LCR, 0x80);
		serial_out(up, UART_MCR, up->mcr | 0x02);
		serial_out(up, UART_LCR, cval);
	}

	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	/* ----Software Flow Control Configuration----- */
	if (termios->c_iflag & (IXON | IXOFF))
		serial_omap_configure_xonxoff(up, termios);

	spin_unlock_irqrestore(&up->port.lock, flags);
	dev_dbg(up->port.dev, "serial_omap_set_termios+%d\n", up->pdev->id);

#ifdef DEBUG
	serial_omap_display_reg(port);
#endif
}

static void
serial_omap_pm(struct uart_port *port, unsigned int state,
	       unsigned int oldstate)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char efr;
	dev_dbg(up->port.dev, "serial_omap_pm+%d\n", up->pdev->id);
	efr = serial_in(up, UART_EFR);
	serial_out(up, UART_LCR, 0xBF);
	serial_out(up, UART_EFR, efr | UART_EFR_ECB);
	serial_out(up, UART_LCR, 0);

	serial_out(up, UART_IER, (state != 0) ? UART_IERX_SLEEP : 0);
	serial_out(up, UART_LCR, 0xBF);
	serial_out(up, UART_EFR, efr);
	serial_out(up, UART_LCR, 0);
	/* Enable module level wake up */
	serial_out(up, UART_OMAP_WER,0x7f);
}

static void serial_omap_release_port(struct uart_port *port)
{
	dev_dbg(port->dev, "serial_omap_release_port+\n");
}

static int serial_omap_request_port(struct uart_port *port)
{
	dev_dbg(port->dev, "serial_omap_request_port+\n");
	return 0;
}

static void serial_omap_config_port(struct uart_port *port, int flags)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	dev_dbg(up->port.dev, "serial_omap_config_port+%d\n",
							up->pdev->id);
	up->port.type = PORT_OMAP;
}

static int
serial_omap_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	dev_dbg(port->dev, "serial_omap_verify_port+\n");
	return -EINVAL;
}

static const char *
serial_omap_type(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	dev_dbg(up->port.dev, "serial_omap_type+%d\n", up->pdev->id);
	return up->name;
}

#ifdef CONFIG_SERIAL_OMAP_CONSOLE

static struct uart_omap_port *serial_omap_console_ports[4];

static struct uart_driver serial_omap_reg;

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

/*
 *	Wait for transmitter & holding register to empty
 */
static inline void wait_for_xmitr(struct uart_omap_port *up)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		if (status & UART_LSR_BI)
			up->lsr_break_flag = UART_LSR_BI;

		if (--tmout == 0)
			break;
		udelay(1);
	} while ((status & BOTH_EMPTY) != BOTH_EMPTY);

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		tmout = 1000000;
		for (tmout = 1000000; tmout; tmout--) {
			unsigned int msr = serial_in(up, UART_MSR);
			up->msr_saved_flags |= msr & MSR_SAVE_FLAGS;
			if (msr & UART_MSR_CTS)
				break;
			udelay(1);
		}
	}
}

static void serial_omap_console_putchar(struct uart_port *port, int ch)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	wait_for_xmitr(up);
	serial_out(up, UART_TX, ch);
}

/*
 * Print a string to the serial port trying not to disturb
 * any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
serial_omap_console_write(struct console *co, const char *s,
		unsigned int count)
{
	struct uart_omap_port *up = serial_omap_console_ports[co->index];
	unsigned int ier;

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_IER);
	serial_out(up, UART_IER, 0);

	uart_console_write(&up->port, s, count, serial_omap_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(up);
	serial_out(up, UART_IER, ier);
	/*
	 *	The receive handling will happen properly because the
	 *	receive ready bit will still be set; it is not cleared
	 *	on read.  However, modem control will not, we must
	 *	call it if we have saved something in the saved flags
	 *	while processing with interrupts off.
	 */
	if (up->msr_saved_flags)
		check_modem_status(up);
}

static int __init
serial_omap_console_setup(struct console *co, char *options)
{
	struct uart_omap_port *up;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int r;

	if (serial_omap_console_ports[co->index] == NULL)
		return -ENODEV;
	up = serial_omap_console_ports[co->index];

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	r = uart_set_options(&up->port, co, baud, parity, bits, flow);

	return r;
}

static struct console serial_omap_console = {
	.name		= DEVICE_NAME,
	.write		= serial_omap_console_write,
	.device		= uart_console_device,
	.setup		= serial_omap_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_omap_reg,
};

static void serial_omap_add_console_port(struct uart_omap_port *up)
{
	serial_omap_console_ports[up->pdev->id - 1] = up;
}

#define OMAP_CONSOLE	(&serial_omap_console)

#else

#define OMAP_CONSOLE	NULL

static inline void serial_omap_add_console_port(struct uart_omap_port *up) {}

#endif

struct uart_ops serial_omap_pops = {
	.tx_empty	= serial_omap_tx_empty,
	.set_mctrl	= serial_omap_set_mctrl,
	.get_mctrl	= serial_omap_get_mctrl,
	.stop_tx	= serial_omap_stop_tx,
	.start_tx	= serial_omap_start_tx,
	.stop_rx	= serial_omap_stop_rx,
	.enable_ms	= serial_omap_enable_ms,
	.break_ctl	= serial_omap_break_ctl,
	.startup	= serial_omap_startup,
	.shutdown	= serial_omap_shutdown,
	.set_termios	= serial_omap_set_termios,
	.pm		= serial_omap_pm,
	.type		= serial_omap_type,
	.release_port	= serial_omap_release_port,
	.request_port	= serial_omap_request_port,
	.config_port	= serial_omap_config_port,
	.verify_port	= serial_omap_verify_port,
};

static struct uart_driver serial_omap_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "OMAP-SERIAL",
	.dev_name	= DEVICE_NAME,
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= OMAP_MAX_HSUART_PORTS,
	.cons		= OMAP_CONSOLE,
};

static
int serial_omap_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct uart_omap_port *up = platform_get_drvdata(pdev);

	if (up)
		uart_suspend_port(&serial_omap_reg, &up->port);
	return 0;
}

static int serial_omap_resume(struct platform_device *dev)
{
	struct uart_omap_port *up = platform_get_drvdata(dev);
	if (up)
		uart_resume_port(&serial_omap_reg, &up->port);
	return 0;
}

static void serial_omap_rx_timeout(unsigned long uart_no)
{
	struct uart_omap_port *up = ui[uart_no - 1];
	unsigned int curr_dma_pos;
	curr_dma_pos = omap_readl(OMAP34XX_DMA4_BASE +
				  OMAP_DMA4_CDAC(up->uart_dma.rx_dma_channel));
	if ((curr_dma_pos == up->uart_dma.prev_rx_dma_pos) ||
			     (curr_dma_pos == 0)) {
		if (jiffies_to_msecs(jiffies - up->port_activity) <
								RX_TIMEOUT) {
			mod_timer(&up->uart_dma.rx_timer, jiffies +
				usecs_to_jiffies(up->uart_dma.rx_timeout));
		} else {
			serial_omap_stop_rxdma(up);
			up->ier |= UART_IER_RDI;
			serial_out(up, UART_IER, up->ier);
		}
		return;
	} else {
		unsigned int curr_transmitted_size = curr_dma_pos -
						up->uart_dma.prev_rx_dma_pos;
		up->port.icount.rx += curr_transmitted_size;
		tty_insert_flip_string(up->port.info->port.tty,
				up->uart_dma.rx_buf +
				(up->uart_dma.prev_rx_dma_pos -
				up->uart_dma.rx_buf_dma_phys),
				curr_transmitted_size);
		tty_flip_buffer_push(up->port.info->port.tty);
		up->uart_dma.prev_rx_dma_pos = curr_dma_pos;
		if (up->uart_dma.rx_buf_size +
				up->uart_dma.rx_buf_dma_phys == curr_dma_pos)
			serial_omap_start_rxdma(up);
		else
			mod_timer(&up->uart_dma.rx_timer, jiffies +
				usecs_to_jiffies(up->uart_dma.rx_timeout));
		up->port_activity = jiffies;
	}
}

static void uart_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	return;
}

static void serial_omap_start_rxdma(struct uart_omap_port *up)
{
	if (up->uart_dma.rx_dma_channel == 0xFF) {
		omap_request_dma(up->uart_dma.uart_dma_rx, "UART Rx DMA",
			(void *)uart_rx_dma_callback, up,
				&(up->uart_dma.rx_dma_channel));

		omap_set_dma_src_params(up->uart_dma.rx_dma_channel, 0,
				OMAP_DMA_AMODE_CONSTANT,
				up->uart_dma.uart_base, 0, 0);
		omap_set_dma_dest_params(up->uart_dma.rx_dma_channel, 0,
				OMAP_DMA_AMODE_POST_INC,
				up->uart_dma.rx_buf_dma_phys, 0, 0);
		omap_set_dma_transfer_params(up->uart_dma.rx_dma_channel,
				OMAP_DMA_DATA_TYPE_S8,
				up->uart_dma.rx_buf_size, 1,
				OMAP_DMA_SYNC_ELEMENT,
				up->uart_dma.uart_dma_rx, 0);
	}
	up->uart_dma.prev_rx_dma_pos = up->uart_dma.rx_buf_dma_phys;
	omap_writel(0, OMAP34XX_DMA4_BASE
			+ OMAP_DMA4_CDAC(up->uart_dma.rx_dma_channel));
	omap_start_dma(up->uart_dma.rx_dma_channel);
	mod_timer(&up->uart_dma.rx_timer, jiffies +
				usecs_to_jiffies(up->uart_dma.rx_timeout));
	up->uart_dma.rx_dma_state = 1;
}

static void serial_omap_continue_tx(struct uart_omap_port *up)
{
	struct circ_buf *xmit = &up->port.info->xmit;
	int start = up->uart_dma.tx_buf_dma_phys
			+ (xmit->tail & (UART_XMIT_SIZE - 1));
	if (uart_circ_empty(xmit))
		return;

	up->uart_dma.tx_buf_size = uart_circ_chars_pending(xmit);
	/* It is a circular buffer. See if the buffer has wounded back.
	 * If yes it will have to be transferred in two separate dma
	 * transfers
	 */
	if (start + up->uart_dma.tx_buf_size >=
			up->uart_dma.tx_buf_dma_phys + UART_XMIT_SIZE)
		up->uart_dma.tx_buf_size =
			(up->uart_dma.tx_buf_dma_phys + UART_XMIT_SIZE) - start;
	omap_set_dma_dest_params(up->uart_dma.tx_dma_channel, 0,
				 OMAP_DMA_AMODE_CONSTANT,
				 up->uart_dma.uart_base, 0, 0);
	omap_set_dma_src_params(up->uart_dma.tx_dma_channel, 0,
		OMAP_DMA_AMODE_POST_INC, start, 0, 0);

	omap_set_dma_transfer_params(up->uart_dma.tx_dma_channel,
				     OMAP_DMA_DATA_TYPE_S8,
				     up->uart_dma.tx_buf_size, 1,
				     OMAP_DMA_SYNC_ELEMENT,
				     up->uart_dma.uart_dma_tx, 0);

	omap_start_dma(up->uart_dma.tx_dma_channel);
}

static void uart_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct uart_omap_port *up = (struct uart_omap_port *)data;
	struct circ_buf *xmit = &up->port.info->xmit;
	xmit->tail = (xmit->tail + up->uart_dma.tx_buf_size) & \
			(UART_XMIT_SIZE - 1);
	up->port.icount.tx += up->uart_dma.tx_buf_size;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit)) {
		spin_lock(&(up->uart_dma.tx_lock));
		serial_omap_stop_tx(&up->port);
		up->uart_dma.tx_dma_state = 0;
		spin_unlock(&(up->uart_dma.tx_lock));
	} else {
		omap_stop_dma(up->uart_dma.tx_dma_channel);
		serial_omap_continue_tx(up);
	}
	up->port_activity = jiffies;
	return;
}

static int serial_omap_probe(struct platform_device *pdev)
{
	struct uart_omap_port	*up;
	struct resource		*mem, *irq, *dma_tx, *dma_rx;
	struct uart_port_info *up_info = pdev->dev.platform_data;
	int ret = -ENOSPC;
	char str[7];

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	ret = (int) request_mem_region(mem->start, (mem->end - mem->start) + 1,
				     pdev->dev.driver->name);
	if (!ret) {
		dev_err(&pdev->dev, "memory region already claimed\n");
		return -EBUSY;
	}

	dma_tx = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!dma_tx) {
		ret = -EINVAL;
		goto err;
	}

	dma_rx = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (!dma_rx) {
		ret = -EINVAL;
		goto err;
	}

	up = kzalloc(sizeof(*up), GFP_KERNEL);
	if (up == NULL) {
		ret = -ENOMEM;
		goto do_release_region;
	}
	sprintf(up->name, "OMAP UART%d", pdev->id);
	up->pdev = pdev;
	up->port.dev = &pdev->dev;
	up->port.type = PORT_OMAP;
	up->port.iotype = UPIO_MEM;
	up->port.mapbase = mem->start;
	up->port.irq = irq->start;

	up->port.fifosize = 64;
	up->port.ops = &serial_omap_pops;
	up->port.line = pdev->id - 1;
	up->port.membase = (void *) OMAP2_IO_ADDRESS(mem->start);
	up->port.flags = UPF_BOOT_AUTOCONF;
	up->port.uartclk = up_info->uartclk;
	up->port.regshift = 2;
	up->uart_dma.uart_base = mem->start;

	if (up_info->dma_enabled) {
		up->uart_dma.uart_dma_tx = dma_tx->start;
		up->uart_dma.uart_dma_rx = dma_rx->start;
		up->use_dma = 1;
		up->uart_dma.rx_buf_size = 4096;
		up->uart_dma.rx_timeout = 1;
	}

	if (up->use_dma) {
		spin_lock_init(&(up->uart_dma.tx_lock));
		spin_lock_init(&(up->uart_dma.rx_lock));
		up->uart_dma.tx_dma_channel = 0xFF;
		up->uart_dma.rx_dma_channel = 0xFF;
	}
	if (console_detect(str)) {
		pr_err("\n %s: Invalid console paramter...\n", __func__);
		pr_err("\n %s: UART Driver Init Failed!\n", __func__);
		return -EPERM;
	}
	ui[pdev->id - 1] = up;
	serial_omap_add_console_port(up);

	ret = uart_add_one_port(&serial_omap_reg, &up->port);
	if (ret != 0)
		goto do_release_region;

	platform_set_drvdata(pdev, up);
	return 0;
err:
	dev_err(&pdev->dev, "[UART%d]: failure [%s]\n",
					pdev->id, __func__);
do_release_region:
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return ret;
}

static int serial_omap_remove(struct platform_device *dev)
{
	struct uart_omap_port *up = platform_get_drvdata(dev);
	platform_set_drvdata(dev, NULL);
	if (up) {
		uart_remove_one_port(&serial_omap_reg, &up->port);
		kfree(up);
	}
	return 0;
}

static struct platform_driver serial_omap_driver = {
	.probe          = serial_omap_probe,
	.remove         = serial_omap_remove,

	.suspend	= serial_omap_suspend,
	.resume		= serial_omap_resume,
	.driver		= {
		.name	= "omap-uart",
	},
};

int __init serial_omap_init(void)
{
	int ret;

	ret = uart_register_driver(&serial_omap_reg);
	if (ret != 0)
		return ret;
	ret = platform_driver_register(&serial_omap_driver);
	if (ret != 0)
		uart_unregister_driver(&serial_omap_reg);
	return ret;
}

void __exit serial_omap_exit(void)
{
	platform_driver_unregister(&serial_omap_driver);
	uart_unregister_driver(&serial_omap_reg);
}


int omap_uart_cts_wakeup(int uart_no, int state)
{
	u32 padconf_cts;
	u16 v;

	unsigned char lcr, efr;
	struct uart_omap_port *up = ui[uart_no];

	if (unlikely(uart_no < 0 || uart_no > OMAP_MAX_HSUART_PORTS)) {
		printk(KERN_INFO "Bad uart id %d \n", uart_no);
		return -EPERM;
	}

	if (state) {
		/*
		 * Enable the CTS for io pad wakeup
		 */
		switch (uart_no) {
		case UART1:
			printk(KERN_INFO "Enabling CTS wakeup for UART1");
			padconf_cts = 0x180;
			v = omap_ctrl_readw(padconf_cts);
			break;
		case UART2:
			printk(KERN_INFO "Enabling CTS wakeup for UART2");
			padconf_cts = 0x174;
			v = omap_ctrl_readw(padconf_cts);
			break;
		default:
			printk(KERN_ERR
			"Wakeup on Uart%d is not supported\n", uart_no);
			return -EPERM;
		}

		v |= ((OMAP3_WAKEUP_EN | OMAP3_OFF_PULL_EN |
			OMAP3_OFFOUT_VAL | OMAP3_OFFOUT_EN |
			OMAP3_OFF_EN | OMAP2_PULL_UP |
			OMAP34XX_MUX_MODE0));

		omap_ctrl_writew(v, padconf_cts);

		/*
		 * Enable the CTS for module level wakeup
		 */
		lcr = serial_in(up, UART_LCR);
		serial_out(up, UART_LCR, 0xbf);
		efr = serial_in(up, UART_EFR);
		serial_out(up, UART_EFR, efr | UART_EFR_ECB);
		serial_out(up, UART_LCR, lcr);
		serial_out(up, UART_OMAP_WER,
				serial_in(up, UART_OMAP_WER) | 0x1);
		serial_out(up, UART_LCR, 0xbf);
		serial_out(up, UART_EFR, efr);
		serial_out(up, UART_LCR, lcr);

	} else {
		/*
		 * Disable the CTS capability for io pad wakeup
		 */
		switch (uart_no) {
		case UART1:
			padconf_cts = 0x180;
			v = omap_ctrl_readw(padconf_cts);
			break;
		case UART2:
			padconf_cts = 0x174;
			v = omap_ctrl_readw(padconf_cts);
			break;
		default:
			printk(KERN_ERR
			"Wakeup on Uart%d is not supported\n", uart_no);
			return -EPERM;
		}

		v &= (u32)(~(OMAP3_WAKEUP_EN | OMAP3_OFF_PULL_EN |
				OMAP3_OFF_EN | OMAP3_OFFOUT_EN));

		omap_ctrl_writew(v, padconf_cts);

		/*
		 * Disable the CTS for module level wakeup
		 */
		lcr = serial_in(up, UART_LCR);
		serial_out(up, UART_LCR, 0xbf);
		efr = serial_in(up, UART_EFR);
		serial_out(up, UART_EFR, efr | UART_EFR_ECB);
		serial_out(up, UART_LCR, lcr);
		serial_out(up, UART_OMAP_WER,
				serial_in(up, UART_OMAP_WER) & ~(0x1));
		serial_out(up, UART_LCR, 0xbf);
		serial_out(up, UART_EFR, efr);
		serial_out(up, UART_LCR, lcr);
	}
	return 0;
}

/**
 * omap_uart_active() - Check if any ports managed by this
 * driver are currently busy.
 */

int omap_uart_active(int num)
{
	struct uart_omap_port *up = ui[num];
	struct circ_buf *xmit;
	unsigned int status;

	/* for DMA mode status of DMA channel
	 * will decide whether uart port can enter sleep
	 * or should we block sleep state.
	 */
	if (up->use_dma &&
		(up->uart_dma.tx_dma_channel != 0xFF ||
		up->uart_dma.rx_dma_channel != 0xFF))
		return 1;
	else
		return 0;

	/* check for recent driver activity */
	/* if from now to last activty < 5 second keep clocks on */
	if ((jiffies_to_msecs(jiffies - up->port_activity) < 5000))
		return 1;

	xmit = &up->port.info->xmit;
	if (!(uart_circ_empty(xmit) || uart_tx_stopped(&up->port)))
		return 1;

	status = serial_in(up, UART_LSR);
	/* TX hardware not empty */
	if (!(status & (UART_LSR_TEMT | UART_LSR_THRE)))
		return 1;

	/* Any rx activity? */
	if (status & UART_LSR_DR)
		return 1;

	/* Any modem activity */
	status = serial_in(up, UART_MSR);
	if (!((status & UART_MSR_ANY_DELTA) == 0))
		return 1;

	return 0;
}
EXPORT_SYMBOL(omap_uart_active);

#ifdef CONFIG_PM
static void omap_uart_save_context(int num)
{
	struct omap_hsuart_state *uart = &omap_hsuart[num];
	struct uart_omap_port *up = ui[num];

	u16 lcr = 0;
	lcr = serial_in(up, UART_LCR);
	serial_out(up, UART_LCR, 0xBF);
	uart->dll = serial_in(up, UART_DLL);
	uart->dlh = serial_in(up, UART_DLM);
	serial_out(up, UART_LCR, lcr);
	uart->ier = serial_in(up, UART_IER);
	uart->sysc = serial_in(up, UART_OMAP_SYSC);
	uart->scr = serial_in(up, UART_OMAP_SCR);
	uart->wer = serial_in(up, UART_OMAP_WER);

	uart->context_valid = 1;
}

static void omap_uart_restore_context(int num)
{
	struct omap_hsuart_state *uart = &omap_hsuart[num];
	struct uart_omap_port *up = ui[num];
	u16 efr = 0;

	if (!uart->context_valid)
		 return;

	uart->context_valid = 0;

	serial_out(up, UART_OMAP_MDR1, 0x7);
	serial_out(up, UART_LCR, 0xBF); /* Config B mode */
	efr = serial_in(up, UART_EFR);
	serial_out(up, UART_EFR, UART_EFR_ECB);
	serial_out(up, UART_LCR, 0x0); /* Operational mode */
	serial_out(up, UART_IER, 0x0);
	serial_out(up, UART_LCR, 0xBF); /* Config B mode */
	serial_out(up, UART_DLL, uart->dll);
	serial_out(up, UART_DLM, uart->dlh);
	serial_out(up, UART_LCR, 0x0); /* Operational mode */
	serial_out(up, UART_IER, uart->ier);
	serial_out(up, UART_FCR, up->fcr);
	serial_out(up, UART_LCR, 0xBF); /* Config B mode */
	serial_out(up, UART_EFR, efr);
	serial_out(up, UART_LCR, UART_LCR_WLEN8);
	serial_out(up, UART_OMAP_SCR, uart->scr);
	serial_out(up, UART_OMAP_WER, uart->wer);
	serial_out(up, UART_OMAP_SYSC, uart->sysc | (0x2 << 3));
	serial_out(up, UART_OMAP_MDR1, 0x00); /* UART 16x mode */
}

/* Remove this during /mach-omap2/serial.c cleanup for 8250 support */
void omap_uart_enable_irqs(int enable) {}

static inline void omap_uart_disable_clocks(int num)
{
	struct omap_hsuart_state *uart = &omap_hsuart[num];
	if (!uart->clocked)
		return;

	omap_uart_save_context(num);
	uart->clocked = 0;
	clk_disable(uart->ick);
	clk_disable(uart->fck);
}

void omap_uart_enable_wakeup(int num)
{
	struct omap_hsuart_state *uart = &omap_hsuart[num];
	/* Set wake-enable bit */
	if (uart->wk_en && uart->wk_mask) {
		u32 v = __raw_readl(uart->wk_en);
		v |= uart->wk_mask;
		__raw_writel(v, uart->wk_en);
	}

	/* Ensure IOPAD wake-enables are set */
	if (cpu_is_omap34xx() && uart->padconf) {
		u16 v = omap_ctrl_readw(uart->padconf);
		v |= OMAP3_PADCONF_WAKEUPENABLE0;
		omap_ctrl_writew(v, uart->padconf);
	}
}

static void omap_uart_smart_idle_enable(int num, int enable)
{
	struct uart_omap_port *up = ui[num];
	u16 sysc;

	sysc = serial_in(up, UART_OMAP_SYSC) & 0x7;

	if (enable)
		/* Errata 2.15: Force idle if in DMA mode */
		sysc |= up->use_dma ? 0x0 : (0x2 << 3);
	else
		sysc |= 0x1 << 3;

	serial_out(up, UART_OMAP_SYSC, sysc);
}

static void omap_uart_block_sleep(int num)
{
	struct omap_hsuart_state *uart = &omap_hsuart[num];

	omap_uart_enable_clocks(num);

	omap_uart_smart_idle_enable(num, 0);
	uart->can_sleep = 0;
	if (uart->timeout)
		mod_timer(&uart->timer, jiffies + uart->timeout);
}

static void omap_uart_allow_sleep(int num)
{
	struct omap_hsuart_state *uart = &omap_hsuart[num];
	if (!uart->clocked)
		return;

	if (uart->padconf)
		omap_uart_enable_wakeup(num);

	omap_uart_smart_idle_enable(num, 1);
	uart->can_sleep = 1;

	if (uart->timeout)
		del_timer_sync(&uart->timer);
}

void omap_uart_prepare_idle(int num)
{
	omap_uart_disable_clocks(num);
}

void omap_uart_resume_idle(int num)
{
	struct omap_hsuart_state *uart = &omap_hsuart[num];

	omap_uart_enable_clocks(num);

	/* Check for IO pad wakeup */
	if (cpu_is_omap34xx() && uart->padconf) {
		u16 p = omap_ctrl_readw(uart->padconf);

	if (p & OMAP3_PADCONF_WAKEUPEVENT0)
		omap_uart_block_sleep(num);
	}

	/* Check for normal UART wakeup */
	if (__raw_readl(uart->wk_st) & uart->wk_mask)
		omap_uart_block_sleep(num);
}

void omap_uart_prepare_suspend(void)
{
	int j;
	for (j = 0; j < OMAP_MAX_HSUART_PORTS; j++)
		omap_uart_allow_sleep(j);
}

int omap_uart_can_sleep(void)
{
	int can_sleep = 1;
	int j;

	for (j = 0; j < OMAP_MAX_HSUART_PORTS; j++) {
		struct omap_hsuart_state *uart = &omap_hsuart[j];

		if (!uart->clocked)
			continue;

		if (!uart->can_sleep) {
			can_sleep = 0;
			continue;
		}

		omap_uart_allow_sleep(j);
	}
	return can_sleep;
}

static void omap_uart_idle_timer(unsigned long data)
{
	struct omap_hsuart_state *uart = (struct omap_hsuart_state *)data;
	if (omap_uart_active(uart->num)) {
		omap_uart_block_sleep(uart->num);
		return;
	}
	omap_uart_allow_sleep(uart->num);
}

void omap_hsuart_idle_init(int num)
{
	struct omap_hsuart_state *uart = &omap_hsuart[num];

	uart->can_sleep = 0;
	uart->timeout = SLEEP_TIMEOUT;
	setup_timer(&uart->timer, omap_uart_idle_timer,
				(unsigned long) uart);

	mod_timer(&uart->timer, jiffies + uart->timeout);

	omap_serial_hsuart_wakeup_source_init(num);
}

#else
static inline void omap_uart_restore_context(int num) {}
static inline void omap_uart_block_sleep(int num) {}
static void omap_uart_smart_idle_enable(int num, int enable) {}
#endif /* CONFIG_PM */

static inline void omap_uart_enable_clocks(int num)
{
	struct omap_hsuart_state *uart = &omap_hsuart[num];
	if (uart->clocked)
		return;

	clk_enable(uart->ick);
	clk_enable(uart->fck);
	uart->clocked = 1;
	omap_uart_restore_context(num);
}

void __init omap_serial_early_init()
{
	int i;
	char name[16];

	for (i = 0; i < OMAP_MAX_HSUART_PORTS; i++) {
		struct omap_hsuart_state *uart = &omap_hsuart[i];
		sprintf(name, "uart%d_ick", i+1);
		uart->ick = clk_get(NULL, name);

		if (IS_ERR(uart->ick)) {
			printk(KERN_ERR "Could not get uart%d_ick\n", i+1);
			uart->ick = NULL;
		}

		sprintf(name, "uart%d_fck", i+1);
		uart->fck = clk_get(NULL, name);

		if (IS_ERR(uart->fck)) {
			printk(KERN_ERR "Could not get uart%d_fck\n", i+1);
			uart->fck = NULL;
		}

		uart->num = i;
		omap_uart_enable_clocks(i);
	}
}

#ifdef DEBUG
#define UART_OMAP_SPR           0x07    /* Scratchpad register */
static void serial_omap_display_reg(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned int lcr, efr, mcr, dll, dlh, xon1, xon2, xoff1, xoff2;
	unsigned int tcr, tlr, uasr;
	pr_debug("Register dump for UART%d\n", up->pdev->id);
	pr_debug("IER_REG = 0x%x\n", serial_in(up, UART_IER));
	pr_debug("IIR_REG = 0x%x\n", serial_in(up, UART_IIR));
	lcr = serial_in(up, UART_LCR);
	pr_debug("LCR_REG = 0x%x\n", lcr);
	mcr = serial_in(up, UART_MCR);
	pr_debug("MCR_REG = 0x%x\n", mcr);
	pr_debug("LSR_REG = 0x%x\n", serial_in(up, UART_LSR));
	pr_debug("MSR_REG = 0x%x\n", serial_in(up, UART_MSR));
	pr_debug("SPR_REG = 0x%x\n", serial_in(up, UART_OMAP_SPR));
	pr_debug("MDR1_REG = 0x%x\n", serial_in(up, UART_OMAP_MDR1));
	pr_debug("MDR2_REG = 0x%x\n", serial_in(up, UART_OMAP_MDR2));
	pr_debug("SCR_REG = 0x%x\n", serial_in(up, UART_OMAP_SCR));
	pr_debug("SSR_REG = 0x%x\n", serial_in(up, UART_OMAP_SSR));
	pr_debug("MVR_REG = 0x%x\n", serial_in(up, UART_OMAP_MVER));
	pr_debug("SYSC_REG = 0x%x\n", serial_in(up, UART_OMAP_SYSC));
	pr_debug("SYSS_REG = 0x%x\n", serial_in(up, UART_OMAP_SYSS));
	pr_debug("WER_REG = 0x%x\n", serial_in(up, UART_OMAP_WER));

	serial_out(up, UART_LCR, 0xBF);
	dll = serial_in(up, UART_DLL);
	dlh = serial_in(up, UART_DLM);
	efr = serial_in(up, UART_EFR);
	xon1 = serial_in(up, UART_XON1);
	xon2 = serial_in(up, UART_XON2);

	serial_out(up, UART_EFR, efr | UART_EFR_ECB);
	serial_out(up, UART_LCR, lcr);
	serial_out(up, UART_MCR, mcr | UART_MCR_TCRTLR);
	serial_out(up, UART_LCR, 0xBF);

	tcr = serial_in(up, UART_TI752_TCR);
	tlr = serial_in(up, UART_TI752_TLR);

	serial_out(up, UART_LCR, lcr);
	serial_out(up, UART_MCR, mcr);
	serial_out(up, UART_LCR, 0xBF);

	xoff1 = serial_in(up, UART_XOFF1);
	xoff2 = serial_in(up, UART_XOFF2);
	uasr = serial_in(up, UART_OMAP_UASR);

	serial_out(up, UART_EFR, efr);
	serial_out(up, UART_LCR, lcr);

	pr_debug("DLL_REG = 0x%x\n", dll);
	pr_debug("DLH_REG = 0x%x\n", dlh);
	pr_debug("EFR_REG = 0x%x\n", efr);

	pr_debug("XON1_ADDR_REG = 0x%x\n", xon1);
	pr_debug("XON2_ADDR_REG = 0x%x\n", xon2);
	pr_debug("TCR_REG = 0x%x\n", tcr);
	pr_debug("TLR_REG = 0x%x\n", tlr);

	pr_debug("XOFF1_REG = 0x%x\n", xoff1);
	pr_debug("XOFF2_REG = 0x%x\n", xoff2);
	pr_debug("UASR_REG = 0x%x\n", uasr);
}
#endif /* DEBUG */

subsys_initcall(serial_omap_init);
module_exit(serial_omap_exit);

MODULE_DESCRIPTION("OMAP High Speed UART driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
