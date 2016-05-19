/*-
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Pseudo driver to copy the NVRAM settings from various sources
 * into the kernel environment.
 *
 * Drivers (such as ethernet devices) can then use environment
 * variables to set default parameters.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include "chipcvar.h"
#include <dev/bhnd/bhnd_debug.h>
#include <dev/nvram2env/nvram2env.h>

#define	NVRAM_MAGIC	0x48534C46
#define NVRAM_MAX_SIZE	0x10000

static int	chipc_otp_probe(device_t dev);
static void	chipc_otp_identify(driver_t *driver, device_t parent);

static void
chipc_otp_identify(driver_t *driver, device_t parent)
{
	struct chipc_caps	*caps;
	device_t		 nvram;

	if (device_find_child(parent, "nvram2env", -1) != NULL)
		return;

	caps = BHND_CHIPC_GET_CAPS(parent);
	if (caps == NULL) {
		device_printf(parent, "no BHND_CHIPC_GET_CAPS found\n");
		return;
	}

	if (caps->otp_size == 0)
		return;

	nvram = BUS_ADD_CHILD(parent, 0, "nvram2env", -1);
}

static int
chipc_otp_probe(device_t dev)
{
	struct nvram2env_softc	*sc;
	struct resource		*res;
	uint32_t		 val;
	uint32_t		 size;

	sc = device_get_softc(dev);
	res = (struct resource*) device_get_ivars(dev);
	if (res == NULL) {
		BHND_ERROR_DEV(dev, "can't retrieve resource for scanning");
		return (ENXIO);
	}

	sc->bst = rman_get_bustag(res);
	size = rman_get_size(res);

	BHND_DEBUG_DEV(dev, "Start scanning flash for NVRAM (%p : %x)",
	    rman_get_virtual(res), size);
	for (uint32_t ofs = 0; ofs < size; ofs+= 0x1000) {
		val = bus_read_4(res, ofs);
		if (val != NVRAM_MAGIC)
			continue;

		/* NVRAM found, stop scanning */
		BHND_TRACE_DEV(dev, "NVRAM found at %x", ofs);
		sc->addr = rman_get_start(res) + ofs;
		sc->maxsize = MIN(NVRAM_MAX_SIZE, size - ofs);
		sc->flags = NVRAM_FLAGS_BROADCOM;
		BHND_TRACE_DEV(dev, "nvram2env probing (addr %x, maxsize %x)",
		    sc->addr, sc->maxsize);
		return (nvram2env_probe(dev));
	}

	BHND_ERROR_DEV(dev, "NVRAM not found");
	return (ENXIO);
}

static device_method_t chipc_otp_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	chipc_otp_identify),
	DEVMETHOD(device_probe,		chipc_otp_probe),
	DEVMETHOD_END
};

DEFINE_CLASS_1(nvram2env, chipc_otp_driver, chipc_otp_methods,
		sizeof(struct nvram2env_softc), nvram2env_driver);

DRIVER_MODULE(chipc_otp, spi, chipc_otp_driver, nvram2env_devclass,
    NULL, NULL);
DRIVER_MODULE(chipc_otp, cfi, chipc_otp_driver, nvram2env_devclass,
    NULL, NULL);

MODULE_VERSION(chipc_otp, 1);
MODULE_DEPEND(chipc_otp, nvram2env, 1, 1, 1);
