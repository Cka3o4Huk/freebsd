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

/*
 * The purpose of ChipCommon BUS is resource management for ChipCommon drivers
 * like UART, PMU, flash. ChipCommon core was a lot of resource:
 *  - several memory regions,
 *  - one or more IRQ lines.
 * To manage / split these resources into resources of drivers this bus
 * is introduced. The bus has 2 resource managers. Driver gets information
 * about BHND ports/regions and map it into resources for drivers.
 *
 * Here is overview of mapping:
 *
 * ------------------------------------------------------
 * | Port.Region| Purpose				|
 * ------------------------------------------------------
 * | 	0.0	| main registers: SPI(0x40), UART(0x300)|
 * |	1.0	| ?					|
 * |	1.1	| MMIO flash (SPI & CFI)		|
 * ------------------------------------------------------
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhndvar.h>
#include <dev/bhnd/bhnd_debug.h>
#include "chipcbus.h"
#include "chipcreg.h"

static struct chipcbus_reg*	chipcbus_get_region(device_t bus,
							int port,
							int regid);
static struct resource_list*	chipcbus_get_resource_list(device_t dev,
							device_t child);
static struct resource*		chipcbus_alloc_resource(device_t bus,
							device_t child,
							int type,
							int *rid,
							rman_res_t start,
							rman_res_t end,
							rman_res_t count,
							u_int flags);
static int	chipcbus_probe(device_t dev);
static int	chipcbus_attach(device_t dev);
static int	chipcbus_detach(device_t dev);
static void	chipcbus_probe_nomatch(device_t dev, device_t child);
static void	chipcbus_set_resources(device_t bus, device_t dev,
		    struct resource_list* rl);
static int	chipcbus_release_resource(device_t bus, device_t child,
		    int type, int rid, struct resource *r);
void		chipcbus_cleanup(device_t dev);

/*
 * There are 2 flash resources:
 *  - resource ID (rid) = 0. This is memory-mapped flash memory
 *  - resource ID (rid) = 1. This is memory-mapped flash registers (i.e for SPI)
 */
struct chipcbus_spec mem_specs[] = {
	{"uart", SYS_RES_MEMORY, 0, 0, 0, CHIPC_UART_BASE, CHIPC_UART_SIZE},
	{"uart", SYS_RES_IRQ, 	 0,-1,-1, 0, 1 },
	{"spi", SYS_RES_MEMORY,  0, 1, 1, 0, ~0},
	{"spi", SYS_RES_MEMORY,  1, 0, 0, CHIPC_FLASHBASE, CHIPC_FLASHREGSZ},
	{"cfi", SYS_RES_MEMORY,  0, 1, 1, 0, ~0},
	{"cfi", SYS_RES_MEMORY,  1, 0, 0, CHIPC_FLASHBASE, CHIPC_FLASHREGSZ},
	{NULL, 0, 0, 0, 0, 0, 0}
};

/*
 * We're creating resource manager for all MIPS IRQs,
 * but it's MIPS-ifed code and it's better to get IRQ number from core info.
 * TODO: fetch IRQ number from core info to demipsify code
 */
struct chipcbus_reg static_irq = { {NULL}, 2, 2, 0, 0 };

static struct chipcbus_reg*
chipcbus_get_region(device_t bus, int port, int regid)
{
	struct chipcbus_ivar	*ivar;
	struct chipcbus_reg	*reg;

	ivar = device_get_ivars(bus);
	SLIST_FOREACH(reg, &ivar->mems, entries){
		if (reg->port == port && reg->reg == regid)
			return (reg);
	}
	return (NULL);
}

static void
chipcbus_set_resources(device_t bus, device_t dev, struct resource_list* rl)
{
	const char		*devname;
	struct chipcbus_spec	*spec;
	struct chipcbus_reg	*reg;

	int		rid;
	int		type;
	rman_res_t	offset;
	rman_res_t	start;
	rman_res_t	end;
	rman_res_t	count;

	BHND_DEBUG_DEV(dev, "first time to init resources...");

	devname = device_get_name(dev);
	spec = mem_specs;
	for (;spec->name != NULL; spec++) {
		if (strcmp(spec->name, devname) != 0)
			continue;

		/* name is matched */
		reg = NULL;
		type = spec->type;
		rid = spec->rid;

		switch (type){
		case SYS_RES_MEMORY:
			reg = chipcbus_get_region(bus, spec->port, spec->reg);
			break;
		case SYS_RES_IRQ:
			reg = &static_irq;
			break;
		default:
			break;
		}

		if (reg == NULL) {
			BHND_WARN_DEV(dev, "region is not found "
			    "[type %d, rid %d, port.region %d.%d]",
			    spec->type, spec->rid, spec->port, spec->reg);
			continue;
		}

		offset = reg->start;
		start = MIN(offset + spec->start, reg->end);
		end = MIN(offset + spec->start +
			  MIN(spec->size, reg->end - reg->start + 1) - 1,
			  reg->end);
		count = end - start + 1;
		resource_list_add(rl,type, rid, start, end, count);
	}
}

void
chipcbus_cleanup(device_t dev)
{
	struct chipcbus_softc *sc;
	sc = device_get_softc(dev);
	rman_fini(&sc->chipc_irq);
	rman_fini(&sc->chipc_mem);
}

static int
chipcbus_probe(device_t dev)
{
	device_set_desc(dev, "ChipCommon BUS");
	return (BUS_PROBE_GENERIC);
}

static int
chipcbus_attach(device_t dev)
{
	int			 err;
	struct chipcbus_softc	*sc;
	struct chipcbus_ivar	*ivar;
	struct chipcbus_reg	*reg;

	BHND_DEBUG_DEV(dev, "start attaching");

	sc = device_get_softc(dev);
	sc->chipc_irq.rm_start = 0;
	sc->chipc_irq.rm_end = NUM_IRQS - 1;
	sc->chipc_irq.rm_type = RMAN_ARRAY;
	sc->chipc_irq.rm_descr = "ChipCommon IRQs";
	err = rman_init(&sc->chipc_irq);
	if (err) {
		BHND_ERROR_DEV(dev, "error occurred during rman_init of "
				"IRQ rman: %d", err);
		goto error;
	}
	sc->chipc_mem.rm_start = 0;
	sc->chipc_mem.rm_end = BUS_SPACE_MAXADDR;
	sc->chipc_mem.rm_type = RMAN_ARRAY;
	sc->chipc_mem.rm_descr = "ChipCommon Memory";
	err = rman_init(&sc->chipc_mem);
	if (err) {
		BHND_ERROR_DEV(dev, "error occurred during init of "
				"MEMORY rman: %d", err);
		goto error;
	}

	ivar = device_get_ivars(dev);
	if (ivar == NULL) {
		BHND_INFO_DEV(dev, "no instance variables for chipcommon bus");
		goto out;
	}

	SLIST_FOREACH(reg, &ivar->mems, entries) {
		err = rman_manage_region(&sc->chipc_mem, reg->start, reg->end);
		if (err) {
			BHND_ERROR_DEV(dev, "error occurred during "
				"rman_manage_region of MEMORY rman with "
				"params: 0x%jx - 0x%jx : err = %d"
				,reg->start, reg->end, err);
			goto error;
		}
	}
/*
 * We're creating resource manager for all MIPS IRQs, but it's MIPS-ifed code
 * and it's better to get IRQ number from core info.
 * TODO: fetch IRQ number from core info to demipsify code
 */
	err = rman_manage_region(&sc->chipc_irq, 0, NUM_IRQS - 1);
	if (err) {
		BHND_ERROR_DEV(dev, "error occurred during "
				"rman_manage_region of IRQ rman with "
				"params: 0x%d - 0x%d : err = %d"
				, 0, NUM_IRQS, err);
		goto error;
	}

out:
	BHND_DEBUG_DEV(dev, "attached successfully");
	return (0);
error:
	BHND_DEBUG_DEV(dev, "attaching finished with error: %d", err);
	chipcbus_cleanup(dev);
	return (err);
}

static int
chipcbus_detach(device_t dev)
{
	chipcbus_cleanup(dev);
	return (0);
}

static struct resource_list *
chipcbus_get_resource_list(device_t dev, device_t child)
{
	struct chipcbus_devinfo *dinfo;

	dinfo = device_get_ivars(child);

	/*
	 * Lazy way of resource assignment
	 */
	if(dinfo == NULL){
		dinfo = malloc(sizeof(struct chipcbus_devinfo*), M_BHND,
				M_NOWAIT);

		if (dinfo == NULL) {
			BHND_ERROR_DEV(dev, "can't allocate memory for "
					"chipcbus_devinfo of %s",
					device_get_nameunit(child));
			return (NULL);
		}

		resource_list_init(&(dinfo->resources));
		chipcbus_set_resources(dev, child, &(dinfo->resources));
		device_set_ivars(child, dinfo);
	}

	return (&dinfo->resources);
}

static struct resource *
chipcbus_alloc_resource(device_t bus, device_t child, int type, int *rid,
	rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct chipcbus_softc		*sc;
	struct rman			*rm;
	struct resource			*r;
	struct resource_list		*rl;
	struct resource_list_entry	*rle;

	BHND_DEBUG_DEV(child,"looking for allocation resource [%d]: %d, "
			"0x%jx, 0x%jx, 0x%jx", type, *rid, start, end, count);
	if (RMAN_IS_DEFAULT_RANGE(start, end) && count == 1) {
		rl = chipcbus_get_resource_list(bus, child);
		if (rl == NULL) {
			BHND_ERROR_DEV(bus, "there is no resource list for: %s",
					device_get_nameunit(child));
			return (NULL);
		}
		rle = resource_list_find(rl, type, *rid);
		if (rle == NULL) {
			BHND_DEBUG_DEV(child, "there is no resource in "
					"resource list: type %d rid %d",
					type, *rid);
			return (NULL);
		}
		start = rle->start;
		end = rle->end;
		count = rle->count;
	}

	sc = device_get_softc(bus);
	switch (type) {
	case SYS_RES_IRQ:
		rm = &sc->chipc_irq;
		break;
	case SYS_RES_MEMORY:
		rm = &sc->chipc_mem;
		break;
	default:
		BHND_ERROR_DEV(child, "unknown resource type %d", type);
		return (NULL);
	}

	BHND_DEBUG_DEV(child, "ready for rman_reserve [%d]: %d, "
			"0x%jx, 0x%jx, 0x%jx", type, *rid, start, end, count);
	r = rman_reserve_resource(rm, start, end, count, flags, child);
	if (r == NULL) {
		BHND_ERROR_DEV(child, "could not reserve resource %d: "
				"0x%jx-0x%jx", *rid, start, end);
		return (NULL);
	}

	rman_set_rid(r, *rid);

	if (flags & RF_ACTIVE)
		if (bus_activate_resource(child, type, *rid, r)) {
			BHND_ERROR_DEV(child, "could not activate resource %d:"
					" 0x%jx-0x%jx", *rid, start, end);
			rman_release_resource(r);
			return (NULL);
		}

	return (r);
}

static int
chipcbus_release_resource(device_t bus, device_t child, int type, int rid,
		       struct resource *r)
{
	int error;

	if (rman_get_flags(r) & RF_ACTIVE) {
		error = bus_deactivate_resource(child, type, rid, r);
		if (error)
			return (error);
	}

	return (rman_release_resource(r));
}

static void
chipcbus_probe_nomatch(device_t dev, device_t child)
{
	device_printf(dev, "no found driver for %s\n", device_get_name(child));
}

static device_method_t chipcbus_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		chipcbus_probe),
	DEVMETHOD(device_attach,	chipcbus_attach),
	DEVMETHOD(device_detach,	chipcbus_detach),

	/* Bus interface */
	DEVMETHOD(bus_get_resource_list,	chipcbus_get_resource_list),
	DEVMETHOD(bus_alloc_resource,		chipcbus_alloc_resource),
	DEVMETHOD(bus_release_resource,		chipcbus_release_resource),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),

	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),
	DEVMETHOD(bus_config_intr,		bus_generic_config_intr),
	DEVMETHOD(bus_bind_intr,		bus_generic_bind_intr),
	DEVMETHOD(bus_describe_intr,		bus_generic_describe_intr),

	DEVMETHOD(bus_probe_nomatch,		chipcbus_probe_nomatch),

	/*
	 * TODO: Add
	 * 	- bus_print_child,
	 * 	- bus_{get,set,delete}_resource,
	 * 	- bus_hinted_child
	 */
	DEVMETHOD_END
};

devclass_t bhnd_chipcbus_devclass;	/* bhnd(4) chipcommon bus device class */

DEFINE_CLASS_0(bhnd_chipcbus, chipcbus_driver, chipcbus_methods,
		sizeof(struct chipcbus_softc));

DRIVER_MODULE(bhnd_chipcbus, bhnd_chipc, chipcbus_driver,
		bhnd_chipcbus_devclass, 0, 0);
MODULE_DEPEND(bhnd_chipcbus, bhnd_chipc, 1, 1, 1);
MODULE_VERSION(bhnd_chipcbus, 1);
