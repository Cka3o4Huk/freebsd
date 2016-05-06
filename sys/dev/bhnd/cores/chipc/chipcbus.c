/*
 * chipcbus.c
 *
 *  Created on: May 5, 2016
 *      Author: mizhka
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * The purpose of ChipCommon BUS is resource management for sub-ChipCommon drivers like UART, PMU, flash.
 * ChipCommon core was a lot of resource: several memory regions, one or more IRQ lines. To manage / split
 * these resources into resources of sub-drivers, this bus is introduced.
 *
 * The bus has 2 resource managers
 * TODO: decide how to split resources
 *
 * Port.Region 	- purpose
 *  0.0 	- main registers (0x40 - SPI, 0x300 - UART)
 *  1.0		- ?
 *  1.1		- MMIO flash (SPI & CFI)
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

#include <ddb/ddb.h>
/*
 * There are 2 flash resources:
 *  - resource ID (rid) = 0. This is memory-mapped flash memory
 *  - resource ID (rid) = 1. This is memory-mapped flash registers (for instance for SPI)
 */
struct chipcbus_spec mem_specs[] = {
	{"uart", SYS_RES_MEMORY, 0, 0, 0, CHIPC_UART_BASE, CHIPC_UART_SIZE},
	{"uart", SYS_RES_IRQ, 	 0,-1,-1, 0, 1 },
	{"spi", SYS_RES_MEMORY,  0, 1, 1, 0, ~0},
	{"spi", SYS_RES_MEMORY,  1, 0, 0, CHIPC_FLASHBASE, CHIPC_FLASHREGSZ},
	{"cfi", SYS_RES_MEMORY,  0, 1, 1, 0, ~0},
	{"cfi", SYS_RES_MEMORY,  1, 0, 0, CHIPC_FLASHBASE, CHIPC_FLASHREGSZ},
	{0, 0, 0, 0, 0, 0, 0}
};

/*
 * FIXME: HACK
 */
struct chipcbus_reg static_irq = { {NULL}, 2, 2, 0, 0 };

static struct chipcbus_reg* chipcbus_get_region(device_t bus, int port, int regid){
	struct chipcbus_ivar* ivar = device_get_ivars(bus);
	struct chipcbus_reg* reg;
	SLIST_FOREACH(reg, &ivar->mems, entries){
		if(reg->port == port && reg->reg == regid){
			return reg;
		}
	}
	return NULL;
}

static void chipcbus_set_resources(device_t bus, device_t dev, struct resource_list* rl){
	BHND_DEBUG_DEV(dev,("first time to init resources..."));
	const char* devname = device_get_name(dev);
	struct chipcbus_spec* spec = mem_specs;
	while(spec->name){
		if(strcmp(spec->name, devname) == 0){
			int type = spec->type;
			int rid = spec->rid;
			struct chipcbus_reg* reg;
			if(type == SYS_RES_MEMORY){
				reg = chipcbus_get_region(bus, spec->port, spec->reg);
			}else if(type == SYS_RES_IRQ){
				reg = &static_irq;
			}
			if(reg != NULL){
				rman_res_t offset = reg->start;
				rman_res_t start = MIN(offset + spec->start, reg->end);
				rman_res_t end = MIN(offset + spec->start + MIN(spec->size, reg->end - reg->start + 1) - 1, reg->end);
				rman_res_t count = end - start + 1;
				resource_list_add(rl,type, rid, start, end, count);
			}else{
				BHND_WARN_DEV(dev,("region is not found [type %d, rid %d, port.region %d.%d]", spec->type, spec->rid, spec->port, spec->reg));
			}
		}
		spec++;
	}
	return;
}

void chipcbus_cleanup(device_t dev){
	struct chipcbus_softc* sc = device_get_softc(dev);
	rman_fini(&sc->chipc_irq);
	rman_fini(&sc->chipc_mem);
	return;
}

static int chipcbus_probe(device_t dev){
	device_set_desc(dev, "ChipCommon BUS");
	return BUS_PROBE_GENERIC;
}

static int chipcbus_attach(device_t dev){
	int err;
	struct chipcbus_softc* sc = device_get_softc(dev);
	BHND_DEBUG_DEV(dev, ("start attaching"));
	sc->chipc_irq.rm_start = 0;
	sc->chipc_irq.rm_end = NUM_IRQS - 1;
	sc->chipc_irq.rm_type = RMAN_ARRAY;
	sc->chipc_irq.rm_descr = "ChipCommon IRQs";
	err = rman_init(&sc->chipc_irq);
	if(err){
		BHND_ERROR_DEV(dev, ("error occurred during init of IRQ rman"));
		return err;
	}
	sc->chipc_mem.rm_start = 0;
	sc->chipc_mem.rm_end = BUS_SPACE_MAXADDR;
	sc->chipc_mem.rm_type = RMAN_ARRAY;
	sc->chipc_mem.rm_descr = "ChipCommon Memory";
	err = rman_init(&sc->chipc_mem);
	if(err){
		BHND_ERROR_DEV(dev, ("error occurred during init of MEMORY rman"));
		goto error;
	}

	struct chipcbus_ivar* ivar = device_get_ivars(dev);
	if(ivar != NULL){
		struct chipcbus_reg* reg;
		SLIST_FOREACH(reg, &ivar->mems, entries){
			err = rman_manage_region(&sc->chipc_mem, reg->start, reg->end);
			if(err){
				BHND_ERROR_DEV(dev, ("error occurred during rman_manage_region of MEMORY rman with params: 0x%jx - 0x%jx"
						,reg->start, reg->end));
				goto error;
			}
		}
//TODO: IRQ
//FIXME: HACK
		err = rman_manage_region(&sc->chipc_irq, 0, NUM_IRQS - 1);
		if(err){
			BHND_ERROR_DEV(dev, ("error occurred during rman_manage_region of IRQ rman with params: 0x%d - 0x%d"
					, 0, NUM_IRQS));
			goto error;
		}
//		for(int i = 0; i < ivar->irq_cnt; i++){
//			err = rman_manage_region(&sc->chipc_mem, ivar->irqs[i].start, ivar->irqs[i].end);
//			if(err){
//				BHND_ERROR_DEV(dev, ("error occurred during rman_manage_region of IRQ rman with params: %jd - %jd"
//						,ivar->mems[i].start, ivar->mems[i].end));
//				goto error;
//			}
//		}
	}else
		BHND_INFO_DEV(dev, ("no instance variables for chipcommon bus"));

	BHND_DEBUG_DEV(dev, ("attached successfully"));
	return 0;
error:
	BHND_DEBUG_DEV(dev, ("attaching finished with error: %d", err));
	chipcbus_cleanup(dev);
	return err;
}

static int chipcbus_detach(device_t dev){
	chipcbus_cleanup(dev);
	return 0;
}

static struct resource_list *
chipcbus_get_resource_list(device_t dev, device_t child){
	struct chipcbus_devinfo *dinfo = device_get_ivars(child);

	/*
	 * Lazy way of resource assignment
	 */
	if(dinfo == NULL){
		dinfo = malloc(sizeof(struct chipcbus_devinfo*), M_BHND, M_NOWAIT);

		if (!dinfo){
			BHND_ERROR_DEV(dev, ("can't allocate memory for chipcbus_devinfo of %s", device_get_nameunit(child)));
			return NULL;
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
	struct chipcbus_softc* sc = device_get_softc(bus);
	BHND_DEBUG_DEV(child,("looking for allocation resource [%d]: %d, 0x%jx, 0x%jx, 0x%jx", type, *rid, start, end, count));

	if(RMAN_IS_DEFAULT_RANGE(start, end) && count == 1){
		struct resource_list* rl = chipcbus_get_resource_list(bus, child);
		struct resource_list_entry* rle = resource_list_find(rl, type, *rid);
		if(rle == NULL){
			BHND_DEBUG_DEV(child, ("there is no resource in resource list: type %d rid %d",type, *rid));
			return NULL;
		}
		start = rle->start;
		end = rle->end;
		count = rle->count;
	}

	struct rman* rm;
	switch (type) {
	case SYS_RES_IRQ:
		rm = &sc->chipc_irq;
		break;
	case SYS_RES_MEMORY:
		rm = &sc->chipc_mem;
		break;
	default:
		BHND_ERROR_DEV(child,("unknown resource type %d", type));
		return NULL;
	}

	BHND_DEBUG_DEV(child,("ready for rman_reserve [%d]: %d, 0x%jx, 0x%jx, 0x%jx", type, *rid, start, end, count));

	struct resource* r = rman_reserve_resource(rm, start, end, count, flags, child);
	if (r == NULL) {
		BHND_ERROR_DEV(child, ("could not reserve resource %d: 0x%jx-0x%jx", *rid, start, end));
		return NULL;
	}

	rman_set_rid(r, *rid);

	if (flags & RF_ACTIVE)
		if (bus_activate_resource(child, type, *rid, r)) {
			BHND_ERROR_DEV(child, ("could not activate resource %d: 0x%jx-0x%jx", *rid, start, end));
			rman_release_resource(r);
			return NULL;
		}

	return r;
}

static int chipcbus_release_resource(device_t bus, device_t child, int type, int rid,
		       struct resource *r)
{
	if (rman_get_flags(r) & RF_ACTIVE) {
		int error = bus_deactivate_resource(child, type, rid, r);
		if (error)
			return error;
	}

	return (rman_release_resource(r));
}

static void chipcbus_probe_nomatch(device_t dev, device_t child){
	device_printf(dev, "no found driver for %s\n", device_get_name(child));
};

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

//	DEVMETHOD(bus_delete_resource,	nexus_delete_resource),
//	DEVMETHOD(bus_get_resource,	nexus_get_resource),
//	DEVMETHOD(bus_print_child,	nexus_print_child),
//	DEVMETHOD(bus_set_resource,	nexus_set_resource),
//	DEVMETHOD(bus_hinted_child,	nexus_hinted_child),

	DEVMETHOD_END
};

devclass_t bhnd_chipcbus_devclass;	/**< bhnd(4) chipcommon bus device class */

DEFINE_CLASS_0(bhnd_chipcbus, chipcbus_driver, chipcbus_methods, sizeof(struct chipcbus_softc));

DRIVER_MODULE(bhnd_chipcbus, bhnd_chipc, chipcbus_driver, bhnd_chipcbus_devclass, 0, 0);
MODULE_DEPEND(bhnd_chipcbus, bhnd_chipc, 1, 1, 1);
MODULE_VERSION(bhnd_chipcbus, 1);
