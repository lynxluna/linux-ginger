/*
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

#include <plat/control.h>
#include <plat/mux.h>

#define DRIVER_NAME	"omap-hsuart"

/* *
 * Use tty device name as ttyO, [O -> OMAP]
 * in bootargs we specify as console=ttyO0 if uart1
 * is used as console uart.
 * Use Major 204 and minor 64.
 * This is necessary if we should coexist with the 8250 driver,
 * if we have an external TI-16C750 UART. Ex.ZOOM2/3 Boards.
 */

#define OMAP_SERIAL_NAME	"ttyO"
#define OMAP_SERIAL_MAJOR	204
#define OMAP_SERIAL_MINOR	64

/* *
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

#define RX_TIMEOUT 	(3 * HZ)

/* To be removed while adding omap_hwmod support */
#define OMAP_MAX_HSUART_PORTS 3

struct uart_port_info {
	bool			dma_enabled;
	unsigned int		uartclk;
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
	/* *
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

extern char *saved_command_line;
int omap_uart_active(int num);
#endif /* __OMAP_SERIAL_H__ */
