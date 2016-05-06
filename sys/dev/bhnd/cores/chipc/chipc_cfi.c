/*
 * chipc_cfi.c
 *
 *  Created on: Feb 29, 2016
 *      Author: mizhka
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <machine/bus.h>

#include <dev/cfi/cfi_var.h>

#include "chipc_slicer.h"

static int
chipc_cfi_probe(device_t dev)
{
	struct cfi_softc *sc = device_get_softc(dev);
	sc->sc_width = 2;
	int error = cfi_probe(dev);
	if (!error)
		device_set_desc(dev, "ChipCommon CFI");
	return (error);
}

static int
chipc_cfi_attach(device_t dev)
{
	int error = cfi_attach(dev);
	if(error)
		return error;

	flash_register_slicer(chipc_slicer_flash);
	return 0;
}

static device_method_t chipc_cfi_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		chipc_cfi_probe),
	DEVMETHOD(device_attach,	chipc_cfi_attach),
	DEVMETHOD(device_detach,	cfi_detach),

	{0, 0}
};

static driver_t chipc_cfi_driver = {
	cfi_driver_name,
	chipc_cfi_methods,
	sizeof(struct cfi_softc),
};

DRIVER_MODULE(cfi, bhnd_chipcbus, chipc_cfi_driver, cfi_devclass, 0, 0);

