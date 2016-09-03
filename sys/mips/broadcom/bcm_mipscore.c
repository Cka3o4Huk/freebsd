/*-
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/rman.h>
#include <sys/stddef.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhndvar.h>
#include <dev/bhnd/bhnd_ids.h>

#include <dev/bhnd/bcma/bcma_dmp.h>

#include "bcm_mipscore.h"

static const struct resource_spec mipscore_rspec[MIPSCORE_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

#define	MIPSCORE_DEV(_vendor, _core)	\
	BHND_DEVICE(_vendor, _core, NULL, NULL, BHND_DF_SOC)

#define	BHND_MIPS74K_IRQN	5
#define	BHND_MIPS74K_IRQBASE	2

struct bhnd_device mipscore_match[] = {
	MIPSCORE_DEV(BCM, MIPS),
	MIPSCORE_DEV(BCM, MIPS33),
	MIPSCORE_DEV(MIPS, MIPS74K),
	BHND_DEVICE_END
};

static int
mipscore_probe(device_t dev)
{
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, mipscore_match, sizeof(mipscore_match[0]));
	if (id == NULL)
		return (ENXIO);

	bhnd_set_default_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}

static int
mipscore_attach(device_t dev)
{
	struct mipscore_softc 	*sc;
	struct resource 	*res;
	device_t		*children;
	uint32_t		 tmp;
	uint32_t 		 intmasks[BHND_MIPS74K_IRQN];
	uint16_t		 devid;
	int			 error;
	int 			 offset, count;
	int			 i, j;

	sc = device_get_softc(dev);
	devid = bhnd_get_device(dev);

	sc->devid = devid;
	sc->dev = dev;

	/* Allocate bus resources */
	memcpy(sc->rspec, mipscore_rspec, sizeof(sc->rspec));
	error = bhnd_alloc_resources(dev, sc->rspec, sc->res);
	if (error)
		return (error);

	res = sc->res[0]->res;
	if (res == NULL)
		return (ENXIO);

	if (devid == BHND_COREID_MIPS74K) {
		tmp = (1 << 31);
		offset = offsetof(struct mipscore_regs, intmask[5]);
		/* Use intmask5 register to route the timer interrupt */
		bus_write_4(res, offset, tmp);
		/* Use intmask0-4 to identify mapping IRQ to cores */
		offset = offsetof(struct mipscore_regs, intmask[0]);

		/* scan IRQ masks of lines */
		for (i = 0; i < BHND_MIPS74K_IRQN; i++) {
			intmasks[i] = bus_read_4(res, offset);
			offset += sizeof(uint32_t);
		}

		error = device_get_children(device_get_parent(dev), &children,
		    &count);

		if (error) {
			BHND_ERROR_DEV(dev, "can't get list of BHND devices: %d",
			    error);
			return (error);
		}

		for (i = 0; i < count; i++) {
			tmp = bhnd_read_config(children[i],
			    BCMA_DMP_OOBSELOUTA30, sizeof(uint32_t));
			if (tmp == 0)
				continue;

			/* filter bits by LINE mask */
			tmp &= 0x1f;
			/* calculate bitmask according to line number */
			tmp = (1 << tmp);

			/* try find line in IRQ bit masks */
			for (j = 0; j < BHND_MIPS74K_IRQN; j++) {
				if ((intmasks[j] & tmp) == 0)
					continue;

				BHND_INFO_DEV(dev, "%s: IRQ %d as rid 0",
				    bhnd_get_device_name(children[i]),
				    BHND_MIPS74K_IRQBASE + j);

				bus_set_resource(children[i], SYS_RES_IRQ, 0,
				    BHND_MIPS74K_IRQBASE + j, 1);

				break;
			}
		}
		free(children, M_TEMP);
	}

	return (0);
}

static device_method_t mipscore_methods[] = {
		DEVMETHOD(device_probe, 	mipscore_probe),
		DEVMETHOD(device_attach,	mipscore_attach),
		DEVMETHOD_END
};

devclass_t bhnd_mipscore_devclass;

DEFINE_CLASS_0(bhnd_mips, mipscore_driver, mipscore_methods,
    sizeof(struct mipscore_softc));
EARLY_DRIVER_MODULE(bhnd_mips, bhnd, mipscore_driver,
    bhnd_mipscore_devclass, 0, 0, BUS_PASS_CPU + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(bhnd_mips, 1);
