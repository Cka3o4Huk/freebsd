/*
 * soc.c
 *
 *  Created on: Feb 16, 2016
 *      Author: mizhka
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/resource.h>

#include <dev/bhnd/bhndvar.h>
#include <dev/bhnd/bhndreg.h>
#include <dev/bhnd/bhndb/bhndb.h>
#include <dev/bhnd/soc/bhnd_soc.h>

#include "bhndb_if.h"

/*
 * **************************** VARIABLES ****************************
 */

struct resource_spec bhnd_soc_default_rspec = {SYS_RES_MEMORY, 0, RF_ACTIVE};

/*
 * **************************** PROTOTYPES ****************************
 */
//internal methods
static int 	bhnd_soc_attach_bus(device_t dev, struct bhnd_soc_softc* sc);
//device methods
static int 	bhnd_soc_probe(device_t dev);
static int 	bhnd_soc_attach(device_t dev);
//bhndb methods
int bhnd_soc_attach_by_class(device_t parent, device_t *child, int unit, devclass_t child_devclass);

/*
 * **************************** IMPLEMENTATION ****************************
 */

int bhnd_soc_attach_by_class(device_t parent, device_t *child, int unit, devclass_t child_devclass)
{
	int error;

	*child = device_add_child(parent, devclass_get_name(child_devclass),
		unit);
	if (*child == NULL)
		return (ENXIO);

	struct bhnd_soc_devinfo* devinfo = malloc(sizeof(struct bhnd_soc_devinfo*), M_BHND, M_NOWAIT);
	resource_list_init(&devinfo->resources);

	for (int i = 0; i < BHND_SOC_MAXNUM_CORES; i++)
		resource_list_add(&devinfo->resources, SYS_RES_MEMORY, i, BHND_SOC_RAM_OFFSET, BHND_SOC_RAM_SIZE, 1);

	device_set_ivars(*child, devinfo);

	if (!(error = device_probe_and_attach(*child)))
		return (0);

	if ((device_delete_child(parent, *child)))
		device_printf(parent, "failed to detach bhndb child\n");

	return (error);
}

static int bhnd_soc_attach_bus(device_t dev, struct bhnd_soc_softc* sc){
	int error = bhnd_read_chipid(dev, &bhnd_soc_default_rspec, BHND_DEFAULT_CHIPC_ADDR, &sc->chipid);

	if (error) {
		return error;
	}

	return bhnd_soc_attach_by_class(dev, &(sc->bus), -1, bhnd_devclass);
}

static int bhnd_soc_probe(device_t dev){
	return BUS_PROBE_GENERIC;
}

static int bhnd_soc_attach(device_t dev){
	struct bhnd_soc_softc* sc = device_get_softc(dev);
	sc->dev = dev;

	return bhnd_soc_attach_bus(dev,sc);
}

static const struct bhnd_chipid * bhnd_soc_get_chipid (device_t dev, device_t child){
	struct bhnd_soc_softc* sc = device_get_softc(dev);
	return &sc->chipid;
}

static  struct resource_list * bhnd_soc_get_rl(device_t dev, device_t child)
{
	struct bhnd_soc_devinfo *dinfo = device_get_ivars(child);
	return (&dinfo->resources);
}

static struct bhnd_resource * bhnd_soc_alloc_resource(device_t dev, device_t child, int type,
     int *rid, rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct bhnd_soc_softc	*sc;
	struct bhnd_resource	*br;
	int error;

	sc = device_get_softc(dev);

	/* Allocate resource wrapper */
	br = malloc(sizeof(struct bhnd_resource), M_BHND, M_NOWAIT|M_ZERO);
	if (br == NULL)
		return (NULL);

	BHND_TRACE_DEV(child,("bus_alloc_resource %d: %jx-%jx (%ju)", *rid, start, end, count));

	/* Configure */
	br->direct = true;
	br->res = bus_alloc_resource(child, type, rid, start, end, count,
	    flags & ~RF_ACTIVE);
	if (br->res == NULL){
		BHND_ERROR_DEV(child,("can't allocate resource %d: %jx-%jx (%ju)", *rid, start, end, count));
		goto failed;
	}

	if (flags & RF_ACTIVE) {
		BHND_TRACE_DEV(child,("bhnd_activate_resource: %d", *rid));
		if ((error = bhnd_activate_resource(child, type, *rid, br))){
			BHND_ERROR_DEV(child,("can't activate BHND resource %d: %jx-%jx (%ju) with error: %d", *rid, start, end, count, error));
			goto failed;
		}
	}

	return (br);

failed:
	if (br->res != NULL)
		bus_release_resource(child, type, *rid, br->res);

	free(br, M_BHND);
	return (NULL);
}

static int bhnd_soc_activate_resource(device_t dev, device_t child, int type,
    int rid, struct bhnd_resource *r){
	/*
	 * Fallback to direct
	 */
	int error = bus_activate_resource(child, type, rid, r->res);
	if(error){
		BHND_ERROR_DEV(child, ("can't activate resource %d with error: %d", rman_get_rid(r->res), error));
		return (error);
	}
	r->direct = true;
	return (0);
}

static bool
bhnd_soc_is_hw_disabled(device_t dev, device_t child){
	return false;
}

static bool
bhnd_soc_is_hostb_device(device_t dev, device_t child){
	return false;
}

/*
 * **************************** DRIVER METADATA ****************************
 */

static device_method_t bhnd_soc_methods[] = {
		//device interface
	DEVMETHOD(device_probe,		bhnd_soc_probe),
	DEVMETHOD(device_attach,	bhnd_soc_attach),
	//resources
	DEVMETHOD(bus_alloc_resource, 		bus_generic_rl_alloc_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),
	DEVMETHOD(bus_set_resource, 		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_release_resource,		bus_generic_rl_release_resource),

	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, 	bus_generic_deactivate_resource),
	//resource list
	DEVMETHOD(bus_get_resource_list,  	bhnd_soc_get_rl),

	//bhnd - BCMA allocates agent resources
	DEVMETHOD(bhnd_bus_alloc_resource,  	bhnd_soc_alloc_resource),
	DEVMETHOD(bhnd_bus_activate_resource,	bhnd_soc_activate_resource),
	DEVMETHOD(bhnd_bus_is_hw_disabled,	bhnd_soc_is_hw_disabled),
	DEVMETHOD(bhnd_bus_is_hostb_device,     bhnd_soc_is_hostb_device),

	//bhndb interface
	DEVMETHOD(bhndb_get_chipid, 		bhnd_soc_get_chipid),
	DEVMETHOD_END
};

devclass_t bhnd_soc_devclass;

DEFINE_CLASS_0(bhnd_soc, bhnd_soc_driver, bhnd_soc_methods, sizeof(struct bhnd_soc_softc));
DRIVER_MODULE(bhnd_soc, nexus, bhnd_soc_driver, bhnd_soc_devclass, NULL, NULL);
