/*
 * b5325hal.c
 *
 *  Created on: May 31, 2016
 *      Author: mizhka
 */


#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * Ported version of bcm5325_switch from ZRouter.org
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/errno.h>
#include <sys/bus.h>

#include "b53_var.h"
#include "b53_reg.h"
#include "b53_hal.h"

static int	b5325hal_get_port_pvid(struct b53_softc *sc, int port, int *pvid);
static int	b5325hal_set_port_pvid(struct b53_softc *sc, int port, int pvid);

struct b53_functions b5325_f = {
	.vlan_get_pvid = b5325hal_get_port_pvid,
	.vlan_set_pvid = b5325hal_set_port_pvid
};

struct b53_hal	b5325_hal = {
	.parent = NULL,
	.own = &b5325_f
};

static int
b5325hal_get_port_pvid(struct b53_softc *sc, int port, int *pvid)
{
	int	local_pvid;

	local_pvid = b53chip_read4(sc, VLAN_DEFAULT_PORT_TAG(port));
	if (local_pvid < 0)
		return (EBUSY);

	*pvid = local_pvid & 0xfff; /* TODO: define macro for mask */
	return (0);
}

int
b5325hal_set_port_pvid(struct b53_softc *sc, int port, int pvid)
{

	if (port > (sc->info.es_nports - 1))
		return (EINVAL);

	if (pvid > 0xfff)
		return (EINVAL);

	return (b53chip_write4(sc, VLAN_DEFAULT_PORT_TAG(port), pvid));
}
