/*
 * arch/arm/plat-omap/include/mach/omap-serial.h
 *
 * Driver for OMAP3430 UART controller.
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Authors:
 *	Govindraj R	<govindraj.raja@ti.com>
 *	Thara Gopinath	<thara@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __OMAP_SERIAL_H__
#define __OMAP_SERIAL_H__

#include <linux/serial_core.h>
#include <linux/platform_device.h>

#include <mach/control.h>
#include <mach/mux.h>

#define DRIVER_NAME	"omap-hsuart"

/* tty device name used by omap-serial driver,
 * in bootargs we specify as console=ttyO0 if uart1
 * is used as console uart.
 */
#define DEVICE_NAME	"ttyO"

/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)  ((irq) != 0)

#if defined(CONFIG_SERIAL_OMAP_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#define OMAP_MDR1_DISABLE 	0x07
#define OMAP_MDR1_MODE13X 	0x03
#define OMAP_MDR1_MODE16X 	0x00
#define OMAP_MODE13X_SPEED	230400

#define CONSOLE_NAME	"console="

#define SLEEP_TIMEOUT 	(5 * HZ)
#define RX_TIMEOUT 	(3 * HZ)

#define UART1		(0x0)
#define UART2		(0x1)
#define UART3		(0x2)

#define OMAP_MAX_HSUART_PORTS 3

struct uart_port_info {
	bool			dma_enabled;
	unsigned int		uartclk;
	};

struct omap_hsuart_state {
	int 			clocked;
	struct 			clk *ick;
	struct			clk *fck;
	int 			num;
	int 			can_sleep;

	void __iomem 		*wk_st;
	void __iomem 		*wk_en;
	u32 			wk_mask;
	u32 			padconf;

#ifdef CONFIG_PM
	struct timer_list 	timer;
	u32 			timeout;
	int 			context_valid;
	/* Registers to be saved/restored for OFF-mode */
	u16 			dll;
	u16 			dlh;
	u16 			ier;
	u16 			sysc;
	u16 			scr;
	u16 			wer;
#endif
};

struct uart_omap_dma {
	u8 			uart_dma_tx;
	u8 			uart_dma_rx;
	int		 	rx_dma_channel;
	int 			tx_dma_channel;
	/* Physical adress of RX DMA buffer */
	dma_addr_t 		rx_buf_dma_phys;
	/* Physical adress of TX DMA buffer */
	dma_addr_t	 	tx_buf_dma_phys;
	/*
	 * Buffer for rx dma.It is not required for tx because the buffer
	 * comes from port structure
	 */
	unsigned int 		uart_base;
	unsigned char 		*rx_buf;
	unsigned int 		prev_rx_dma_pos;
	int 			tx_buf_size;
	int 			tx_dma_state;
	int 			rx_dma_state;
	spinlock_t 		tx_lock;
	spinlock_t 		rx_lock;
	/* timer to poll activity on rx dma */
	struct timer_list 	rx_timer;
	int 			rx_buf_size;
	int 			rx_timeout;
};

struct uart_omap_port {
	struct uart_port	port;
	struct uart_omap_dma	uart_dma;
	struct platform_device	*pdev;

	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		fcr;
	unsigned char		efr;

	int			use_dma;
	int			is_buf_dma_alloced;
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
	unsigned int		lsr_break_flag;
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;
	char			name[20];
	spinlock_t		uart_lock;
	unsigned long 		port_activity;
};

int omap_uart_cts_wakeup(int uart_no, int state);

extern void omap_hsuart_idle_init(int num);
extern void omap_serial_hsuart_wakeup_source_init(int num);

extern struct omap_hsuart_state omap_hsuart[OMAP_MAX_HSUART_PORTS];
extern char *saved_command_line;
#endif /* __OMAP_SERIAL_H__ */
