/*-
 * Copyright (c) 2011-2012 Stefan Bethke.
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
 *
 * $FreeBSD$
 */

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <dev/mdio/mdio.h>

#include "mdio_if.h"

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/fdt/simplebus.h>

/* OFW bus interface */
struct mdio_ofw_devinfo {
	struct ofw_bus_devinfo	di_dinfo;
	struct resource_list	di_rl;
};

#endif /* FDT */

static void
mdio_identify(driver_t *driver, device_t parent)
{

	if (device_find_child(parent, mdio_driver.name, -1) == NULL)
		BUS_ADD_CHILD(parent, 0, mdio_driver.name, -1);
}

static int
mdio_probe(device_t dev)
{

	device_set_desc(dev, "MDIO");

	return (BUS_PROBE_SPECIFIC);
}

#ifdef FDT
static void				mdio_add_device(device_t dev,
					    phandle_t node, phandle_t child);
static struct mdio_ofw_devinfo *	mdio_setup_dinfo(device_t dev,
					    phandle_t node, phandle_t child);
static const struct ofw_bus_devinfo *	mdio_ofw_get_devinfo(device_t bus,
					    device_t child);

static void
mdio_add_device(device_t dev, phandle_t node, phandle_t child)
{
	struct mdio_ofw_devinfo	*ndi;
	device_t		 cdev;

	ndi = mdio_setup_dinfo(dev, node, child);
	if (ndi == NULL)
		return;

	cdev = device_add_child_ordered(dev, 0, NULL, -1);
	if (cdev == NULL) {
		device_printf(dev, "<%s>: device_add_child failed\n",
		    ndi->di_dinfo.obd_name);
		resource_list_free(&ndi->di_rl);
		ofw_bus_gen_destroy_devinfo(&ndi->di_dinfo);
		free(ndi, M_DEVBUF);
		return;
	}

	device_set_ivars(cdev, ndi);
}

static struct mdio_ofw_devinfo *
mdio_setup_dinfo(device_t dev, phandle_t node, phandle_t child)
{
	struct mdio_ofw_devinfo	*ndi;
	pcell_t			 acells, scells;

	acells = 2;
	scells = 1;

	ndi = malloc(sizeof(*ndi), M_DEVBUF, M_WAITOK | M_ZERO);

	if (ofw_bus_gen_setup_devinfo(&ndi->di_dinfo, child) != 0) {
		free(ndi, M_DEVBUF);
		return (NULL);
	}

	resource_list_init(&ndi->di_rl);

	OF_getencprop(node, "#address-cells", &acells, sizeof(acells));
	OF_getencprop(node, "#size-cells", &scells, sizeof(scells));

	ofw_bus_reg_to_rl(dev, child, acells, scells, &ndi->di_rl);
	ofw_bus_intr_to_rl(dev, child, &ndi->di_rl, NULL);

	return (ndi);
}

static const struct ofw_bus_devinfo *
mdio_ofw_get_devinfo(device_t bus __unused, device_t child)
{
	struct mdio_ofw_devinfo *di;

	di = device_get_ivars(child);
	return (&di->di_dinfo);
}

#endif /* FDT */

static int
mdio_attach(device_t dev)
{
#ifdef FDT
	phandle_t 	 node;
	phandle_t	 child;
	const char	*name;

	node = ofw_bus_get_node(dev);
	name = ofw_bus_get_name(dev);

	device_printf(dev, "looking for ofw children for %s / %x\n", name, node);
	/* Attach child devices */
	for (child = OF_child(node); child > 0; child = OF_peer(child))
		mdio_add_device(dev, node, child);
#endif
	bus_generic_probe(dev);
	bus_enumerate_hinted_children(dev);
	return (bus_generic_attach(dev));
}

static int
mdio_detach(device_t dev)
{

	bus_generic_detach(dev);
	return (0);
}

static int
mdio_readreg(device_t dev, int phy, int reg)
{

	return (MDIO_READREG(device_get_parent(dev), phy, reg));
}

static int
mdio_writereg(device_t dev, int phy, int reg, int val)
{

	return (MDIO_WRITEREG(device_get_parent(dev), phy, reg, val));
}

static int
mdio_readextreg(device_t dev, int phy, int devad, int reg)
{

	return (MDIO_READEXTREG(device_get_parent(dev), phy, devad, reg));
}

static int
mdio_writeextreg(device_t dev, int phy, int devad, int reg,
    int val)
{

	return (MDIO_WRITEEXTREG(device_get_parent(dev), phy, devad, reg, val));
}

static void
mdio_hinted_child(device_t dev, const char *name, int unit)
{

	device_add_child(dev, name, unit);
}

static device_method_t mdio_methods[] = {
	/* device interface */
	DEVMETHOD(device_identify,	mdio_identify),
	DEVMETHOD(device_probe,		mdio_probe),
	DEVMETHOD(device_attach,	mdio_attach),
	DEVMETHOD(device_detach,	mdio_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	/* bus interface */
	DEVMETHOD(bus_add_child,	device_add_child_ordered),
	DEVMETHOD(bus_hinted_child,	mdio_hinted_child),

	/* MDIO access */
	DEVMETHOD(mdio_readreg,		mdio_readreg),
	DEVMETHOD(mdio_writereg,	mdio_writereg),
	DEVMETHOD(mdio_readextreg,	mdio_readextreg),
	DEVMETHOD(mdio_writeextreg,	mdio_writeextreg),

#ifdef FDT
	DEVMETHOD(ofw_bus_get_devinfo,	mdio_ofw_get_devinfo),
	DEVMETHOD(ofw_bus_get_compat,	ofw_bus_gen_get_compat),
	DEVMETHOD(ofw_bus_get_model,	ofw_bus_gen_get_model),
	DEVMETHOD(ofw_bus_get_name,	ofw_bus_gen_get_name),
	DEVMETHOD(ofw_bus_get_node,	ofw_bus_gen_get_node),
	DEVMETHOD(ofw_bus_get_type,	ofw_bus_gen_get_type),
#endif

	DEVMETHOD_END
};

driver_t mdio_driver = {
	"mdio",
	mdio_methods,
	0
};

static int
mdio_modevent(module_t mod, int type, void *data)
{

	switch (type) {
	case MOD_LOAD:
		break;
	case MOD_UNLOAD:
		break;
	default:
		return (EOPNOTSUPP);
	}
	return (0);
}

static moduledata_t mdio_mod = {
	"mdio",
	mdio_modevent,
	0
};

devclass_t mdio_devclass;

DECLARE_MODULE(mdio, mdio_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_VERSION(mdio, 1);
