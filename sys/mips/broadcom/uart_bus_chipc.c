/*
 * uart_bus_chipc.c
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
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_cpu.h>

#include "uart_if.h"

static int uart_chipc_probe(device_t dev);

extern SLIST_HEAD(uart_devinfo_list, uart_devinfo) uart_sysdevs;
static int
uart_chipc_probe(device_t dev)
{
	struct uart_softc *sc;

	sc = device_get_softc(dev);

	int rid = 0;
	struct resource* res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if(res == NULL){
		device_printf(dev, "can't allocate main resource\n");
		return (ENXIO);
	}

	sc->sc_class = &uart_ns8250_class;
	sc->sc_sysdev = SLIST_FIRST(&uart_sysdevs);
	if (sc->sc_sysdev == NULL){
		device_printf(dev, "missing sysdev\n");
		return (EINVAL);
	}

	bcopy(&sc->sc_sysdev->bas, &sc->sc_bas, sizeof(sc->sc_bas));

	sc->sc_sysdev->bas.bst = rman_get_bustag(res);
	sc->sc_sysdev->bas.bsh = rman_get_bushandle(res);
	sc->sc_bas.bst = sc->sc_sysdev->bas.bst;
	sc->sc_bas.bsh = sc->sc_sysdev->bas.bsh;

	bus_release_resource(dev, SYS_RES_MEMORY, rid, res);

	//We use internal SoC clock generator with non-standart freq MHz
	return(uart_bus_probe(dev, 0, sc->sc_sysdev->bas.rclk, 0, 0));
}

static device_method_t uart_chipc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		uart_chipc_probe),
	DEVMETHOD(device_attach,	uart_bus_attach),
	DEVMETHOD(device_detach,	uart_bus_detach),
	{ 0, 0 }
};

static driver_t uart_chipc_driver = {
	uart_driver_name,
	uart_chipc_methods,
	sizeof(struct uart_softc),
};

DRIVER_MODULE(uart, bhnd_chipcbus, uart_chipc_driver, uart_devclass, 0, 0);
