 
/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
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
 * Generic BHND PCI Device Support
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "bhndb_private.h"

#include "bhndb_bus_if.h"

#include "bhndb_pcireg.h"
#include "bhndb_pcivar.h"
#include "bhndb_private.h"

int
bhndb_pci_probe(device_t dev)
{
	device_t	parent;
	devclass_t	parent_bus;
	devclass_t	pci;

	/* Our parent must be a PCI device. */
	pci = devclass_find("pci");
	parent = device_get_parent(dev);
	parent_bus = device_get_devclass(device_get_parent(parent));

	if (parent_bus != pci) {
		device_printf(dev, "bhndb_pci attached to non-PCI parent %s\n",
		    device_get_nameunit(parent));
		return (ENXIO);
	}
	
	return (BUS_PROBE_NOWILDCARD);
}

static bool
bhndb_pci_hw_matches(device_t dev, struct bhnd_core_info *cores,
    u_int num_cores, const struct bhndb_hw *hw)
{
	for (u_int i = 0; i < hw->num_hw_reqs; i++) {
		const struct bhnd_core_match *match =  &hw->hw_reqs[i];

		if (bhnd_match_core(cores, num_cores, match) == NULL)
			return (false);
	}

	return (true);
}

static int
bhndb_pci_find_hwcfg(device_t dev, const struct bhndb_hwcfg **hwcfg)
{
	struct bhndb_pci_softc	*sc;
	const struct bhndb_hw	*hw;
	struct bhnd_core_info	*cores;
	u_int			 num_cores;
	int			 error;

	sc = device_get_softc(dev);
	error = 0;

	/* Fetch our core table */
	if ((error = BHNDB_GET_CORE_TABLE(dev, &cores, &num_cores)))
		return (error);
	
	/* Search for the first matching hardware config */
	for (hw = bhndb_pci_hw; hw->hw_reqs != NULL; hw++) {
		if (bhndb_pci_hw_matches(dev, cores, num_cores, hw)) {
			device_printf(dev, "Register Map: %s\n",
			    hw->name);
			*hwcfg = hw->cfg;
			goto finished;
		}
	}

	/* Not found */
	error = ENOENT;

finished:
	free(cores, M_BHND);
	return (error);
}

int
bhndb_pci_attach(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error;
	bool			 free_mem_rman = false;
	bool			 free_pci_res = false;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->pci_dev = device_get_parent(dev);
	
	sc->res = NULL;
	sc->res_spec = NULL;

	/* Determine our PCI/ChipCommon register window configuration */
	if ((error = bhndb_pci_find_hwcfg(dev, &sc->hwcfg)))
		return (error);

	/* Set up a resource manager for the device's address space. */
	sc->mem_rman.rm_start = 0;
	sc->mem_rman.rm_end = BUS_SPACE_MAXADDR_32BIT;
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "bhnd device address space";
	
	if (rman_init(&sc->mem_rman) ||
	    rman_manage_region(&sc->mem_rman, 0, BUS_SPACE_MAXADDR_32BIT))
	{
		device_printf(dev, "could not initialize mem_rman\n");
		return (ENXIO);
	} else {
		free_mem_rman = true;
	}

	/* Determine our PCI resource count from the hardware config. */
	sc->res_count = 0;
	for (size_t i = 0; sc->hwcfg->resource_specs[i].type != -1; i++)
		sc->res_count++;
	
	/* Allocate space for a non-const copy of our resource_spec
	 * table; this will be updated with the RIDs assigned by
	 * bus_alloc_resources. */
	sc->res_spec = malloc(sizeof(struct resource_spec) * (sc->res_count + 1),
	    M_BHND, M_WAITOK);
	if (sc->res_spec == NULL) {
		error = ENOMEM;
		goto failed;
	}

	/* Initialize and terminate the table */
	for (size_t i = 0; i < sc->res_count; i++)
		sc->res_spec[i] = sc->hwcfg->resource_specs[i];
	
	sc->res_spec[sc->res_count].type = -1;

	/* Allocate space for our resource references */
	sc->res = malloc(sizeof(struct resource) * sc->res_count,
	    M_BHND, M_WAITOK);
	if (sc->res == NULL) {
		error = ENOMEM;
		goto failed;
	}

	/* Allocate resources */
	error = bus_alloc_resources(sc->pci_dev, sc->res_spec, sc->res);
	if (error) {
		device_printf(dev,
		    "could not allocate PCI resources on %s: %d\n",
		    device_get_nameunit(sc->pci_dev), error);
		goto failed;
	} else {
		free_pci_res = true;
	}

	return (bus_generic_attach(dev));

failed:
	if (free_mem_rman)
		rman_fini(&sc->mem_rman);

	if (free_pci_res)
		bus_release_resources(dev, sc->res_spec, sc->res);

	if (sc->res != NULL)
		free(sc->res, M_BHND);

	if (sc->res_spec != NULL)
		free(sc->res_spec, M_BHND);

	return (error);
}

int
bhndb_pci_detach(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);
	
	/* Detach children */
	error = bus_generic_detach(dev);

	/* Clean up */
	rman_fini(&sc->mem_rman);

	bus_release_resources(dev, sc->res_spec, sc->res);
	free(sc->res, M_BHND);
	free(sc->res_spec, M_BHND);

	return (error);
}

int
bhndb_pci_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

int
bhndb_pci_resume(device_t dev)
{
	return (bus_generic_resume(dev));
}

int
bhndb_pci_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	switch (index) {
	default:
		return (ENOENT);
	}
}

int
bhndb_pci_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
{
	switch (index) {
	default:
		return (ENOENT);
	}
}

static struct rman *
bhndb_pci_get_rman(device_t dev, int type)
{
	struct bhndb_pci_softc *sc = device_get_softc(dev);

	switch (type) {
	case SYS_RES_MEMORY:
		return &sc->mem_rman;
	case SYS_RES_IRQ:
		// TODO
		// return &sc->irq_rman;
		return (NULL);
	default:
		return (NULL);
	};
}

/**
 * Helper function for implementing BUS_ALLOC_RESOURCE() on bhnd pci hosts.
 * 
 * This simple implementation uses BHND_GET_RMAN() and BUS_GET_RESOURCE_LIST()
 * to fetch resource state for allocation.
 */
static struct resource *
bhndb_pci_alloc_resource(device_t dev, device_t child, int type,
    int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	struct resource_list		*rl;
	struct resource_list_entry	*rle;
	struct rman			*rm;
	int				 error;

	if (device_get_parent(child) != dev) {
		return (BUS_ALLOC_RESOURCE(device_get_parent(dev), child,
		    type, rid, start, end, count, flags));
	}

	/* Fetch the resource manager */
	rm = bhndb_pci_get_rman(dev, type);
	if (rm == NULL)
		return (NULL);

	/* Fetch the resource list entry */
	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl == NULL)
		return (NULL);

	rle = resource_list_find(rl, type, *rid);
	if (rle == NULL)
		return (NULL);

	/* Validate the resource addresses */
	if (start == 0ULL && end == ~0ULL && count <= rle->count) {
		start = rle->start;
		end = rle->end;
		count = rle->count;
	} else {
		if (start < rle->start || start > end)
			return NULL;
		
		if (end < start || end > rle->end)
			return NULL;
		
		if (count > end - start)
			return NULL;
	}

	/* Check for existing allocation */
	if (rle->res != NULL) {
		device_printf(dev,
		    "resource entry %#x type %d for child %s is busy\n", *rid,
			type, device_get_nameunit(child));
		return (NULL);
	}

	/* Make our reservation */
	rle->res = rman_reserve_resource(rm, start, end, count, flags, child);
	if (rle->res == NULL)
		return (NULL);

	rman_set_rid(rle->res, *rid);

	/* Activate */
	if (flags & RF_ACTIVE) {
		error = bus_activate_resource(child, type, *rid, rle->res);
		if (error) {
			device_printf(dev,
			    "failed to activate entry %#x type %d for "
				"child %s\n",
			     *rid, type, device_get_nameunit(child));

			rman_release_resource(rle->res);
			rle->res = NULL;
			return (NULL);
		}
	}

	return (rle->res);
}

/**
 * Helper function for implementing BUS_RELEASE_RESOURCE() on bhnd pci hosts.
 * 
 * This simple implementation uses BUS_GET_RESOURCE_LIST() to fetch resource
 * state.
 */
static int
bhndb_pci_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *res)
{
	struct resource_list		*rl;
	struct resource_list_entry	*rle;
	int				 error;

	if (device_get_parent(child) != dev) {
		return (BUS_RELEASE_RESOURCE(device_get_parent(dev), child,
		    type, rid, res));
	}

	/* Fetch the resource list entry */
	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl == NULL)
		return (EINVAL);

	rle = resource_list_find(rl, type, rid);
	if (rle == NULL)
		panic("missing resource list entry");

	if (rle->res != res)
		panic("resource list entry does not match resource");

	/* Release the resource */
	if ((error = rman_release_resource(res)))
		return (error);

	rle->res = NULL;
	return (0);
}

/**
 * Helper function for implementing BUS_ACTIVATE_RESOURCE() on bhnd pci hosts.
 */
static int
bhndb_pci_activate_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	// TODO - window resource activations
	return (EINVAL);
}

/**
 * Helper function for implementing BUS_ACTIVATE_RESOURCE() on bhnd pci hosts.
 */
static int
bhndb_pci_deactivate_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	// TODO - window resource deactivations
	return (EINVAL);
}

/**
 * Helper function for implementing BHND_ALLOC_RESOURCE().
 * 
 * This simple implementation delegates allocation of the backing resource
 * to BUS_ALLOC_RESOURCE().
 */
static struct bhnd_resource *
bhndb_pci_alloc_bhnd_resource(device_t dev, device_t child, int type,
     int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	// struct bhnd_resource *r;
	
	if (device_get_parent(child) != dev)
		return (BHND_ALLOC_RESOURCE(device_get_parent(dev), child,
		    type, rid, start, end, count, flags));

	// TODO
	return (NULL);
}

/**
 * Helper function for implementing BHND_RELEASE_RESOURCE().
 */
static int
bhndb_pci_release_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	// int error;
	
	if (device_get_parent(child) != dev)
		return (BHND_RELEASE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	// TODO
	return (EOPNOTSUPP);
}

/**
 * Helper function for implementing BHND_ACTIVATE_RESOURCE().
 * 
 * This simple implementation delegates allocation of the backing resource
 * to BUS_ACTIVATE_RESOURCE().
 */
static int
bhndb_pci_activate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	if (device_get_parent(child) != dev)
		return (BHND_ACTIVATE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	// TODO
	return (EOPNOTSUPP);
};

/**
 * Helper function for implementing BHND_DEACTIVATE_RESOURCE().
 * 
 * This simple implementation delegates allocation of the backing resource
 * to BUS_DEACTIVATE_RESOURCE().
 */
static int
bhndb_pci_deactivate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	if (device_get_parent(child) != dev)
		return (BHND_DEACTIVATE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	// TODO
	return (EOPNOTSUPP);
};


static device_method_t bhndb_pci_methods[] = {
	/* Device interface */ \
	DEVMETHOD(device_probe,			bhndb_pci_probe),
	DEVMETHOD(device_attach,		bhndb_pci_attach),
	DEVMETHOD(device_detach,		bhndb_pci_detach),
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),
	DEVMETHOD(device_suspend,		bhndb_pci_suspend),
	DEVMETHOD(device_resume,		bhndb_pci_resume),

	/* Bus interface */
	DEVMETHOD(bus_alloc_resource,		bhndb_pci_alloc_resource),
	DEVMETHOD(bus_release_resource,		bhndb_pci_release_resource),
	DEVMETHOD(bus_activate_resource,	bhndb_pci_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bhndb_pci_deactivate_resource),
	DEVMETHOD(bus_read_ivar,		bhndb_pci_read_ivar),
	DEVMETHOD(bus_write_ivar,		bhndb_pci_write_ivar),

	/* BHND interface */
	DEVMETHOD(bhnd_alloc_resource,		bhndb_pci_alloc_bhnd_resource),
	DEVMETHOD(bhnd_release_resource,	bhndb_pci_release_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhndb_pci_activate_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhndb_pci_deactivate_bhnd_resource),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhndb, bhndb_pci_driver, bhndb_pci_methods, sizeof(struct bhndb_pci_softc));

MODULE_VERSION(bhndb_pci, 1);

MODULE_DEPEND(bhndb_pci, pci, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, bhnd, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, bhndb, 1, 1, 1);
