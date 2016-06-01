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

#include "b53switch.h"
#include "b53switchvar.h"
#include "b53switchreg.h"
#include "b53hal.h"

int
b5325hal_getvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct b53switch_softc	*sc;
	uint16_t		 vid;
	uint32_t		 reg;
	int			 error;

	sc = device_get_softc(dev);

	error = 0;
	vid = vg->es_vlangroup;
	reg = VLAN_TABLE_ACCESS_RW_ENABLE |
	    (vid & VLAN_TABLE_ACCESS_VID_MASK);
	error = b53switch_write4(sc, VLAN_TABLE_ACCESS, reg);
	if (error) {
		device_printf(dev, "can't write to VLAN_TABLE_ACCESS reg: %d\n",
		    error);
		return (error);
	}

	error = b53switch_op(sc, VLAN_READ, &reg, 0);
	if (error) {
		device_printf(dev, "can't read from VLAN_READ reg: %d\n",
		    error);
		return (error);
	}

	if (!(reg & VLAN_RW_VALID_5358)) {
		device_printf(dev, "not a valid VLAN id [%d] = 0x%x\n",
		    vid, reg);
		return (ENOENT);
	}
}
