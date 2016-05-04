/*
 * uart_cpu_chipc.c
 *
 *  Created on: Apr 3, 2016
 *      Author: mizhka
 */


#include "opt_uart.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cons.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <machine/bus.h>

#include <sys/kdb.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_cpu.h>

bus_space_tag_t uart_bus_space_io;
bus_space_tag_t uart_bus_space_mem;

int
uart_cpu_eqres(struct uart_bas *b1, struct uart_bas *b2)
{
	return ((b1->bsh == b2->bsh && b1->bst == b2->bst) ? 1 : 0);
}

int
uart_cpu_getdev(int devtype, struct uart_devinfo *di)
{
	struct uart_class *class;

//	if(devtype == UART_DEV_CONSOLE)
//		return 1;

	class = &uart_ns8250_class;
//	if (class->uc_rclk == 0 && at91_master_clock != 0)
//		class->uc_rclk = at91_master_clock;
	di->ops = uart_getops(class);
	di->bas.chan = 0;
	di->bas.bst = mips_bus_space_generic;
	di->bas.bsh = 0xb8000300;//soc_info.dbgu_base;
	di->bas.regshft = 0;
	di->bas.rclk = 20000000; // depends on SoC, it can be 20MHz or 25MHz
	di->baudrate = 115200;
	di->databits = 8;
	di->stopbits = 1;
	di->parity = UART_PARITY_NONE;
	uart_bus_space_io = NULL;
	uart_bus_space_mem = mips_bus_space_generic;
	/* Check the environment for overrides */
	//uart_getenv(devtype, di, class);
	return (0);
}
