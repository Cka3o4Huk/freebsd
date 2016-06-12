/*
 * b53switch_hal.c
 *
 *  Created on: Jun 8, 2016
 *      Author: mizhka
 */


#include <sys/types.h>
#include <sys/bus.h>
#include <sys/errno.h>

#include "b53_var.h"
#include "b53_reg.h"
#include "b53_hal.h"

static int	b5358hal_get_vlan_group(struct b53_softc *sc, int vlan_group,
		    int *vlan_id, int *members, int *untagged, int *forward_id);
static int	b5358hal_set_vlan_group(struct b53_softc *sc, int vlan_group,
		    int vlan_id, int members, int untagged, int forward_id);

struct b53_functions b5358_f = {
	.vlan_get_vlan_group = b5358hal_get_vlan_group,
	.vlan_set_vlan_group = b5358hal_set_vlan_group
};

struct b53_hal b5358_hal = {
	.parent = &b5325_hal,
	.own = &b5358_f
};

static int
b5358hal_get_vlan_group(struct b53_softc *sc, int vlan_group,
		    int *vlan_id, int *members, int *untagged, int *forward_id)
{
	uint32_t		 reg;
	int			 error;

	/* Set VLAN_GROUP as input */
	reg = VLAN_TABLE_ACCESS_RW_ENABLE;
	reg|= vlan_group & VLAN_TABLE_ACCESS_VID_MASK;
	error = b53chip_write4(sc, VLAN_TABLE_ACCESS, reg);
	if (error) {
		device_printf(sc->sc_dev, "can't write to VLAN_TABLE_ACCESS"
		    "[%d]: %d\n", vlan_group, error);
		return (error);
	}

	/* Read VLAN information */
	error = b53chip_op(sc, VLAN_READ, &reg, 0);
	if (error) {
		device_printf(sc->sc_dev, "can't read from VLAN_READ[%d]: %d\n",
		    vlan_group, error);
		return (error);
	}

	/* Check if it valid */
	if (!(reg & VLAN_RW_VALID_5358)) {
		device_printf(sc->sc_dev, "not a valid VLAN[%d] info = 0x%x\n",
		    vlan_group, reg);
		return (ENOENT);
	}

#if 0
	printf("reg[%d] = 0x%x\n",vlan_group, reg);
#endif
	*vlan_id = ETHERSWITCH_VID_VALID | vlan_group;
	*members = B53_UNSHIFT(reg, VLAN_RW_MEMBER);
	*untagged = B53_UNSHIFT(reg, VLAN_RW_UNTAGGED);
	/* TODO: forwarding */
	*forward_id = 0;

	return (0);
}

static int
b5358hal_set_vlan_group(struct b53_softc *sc, int vlan_group,
		    int vlan_id, int members, int untagged, int forward_id)
{
	uint32_t		 reg;
	int			 error;

	/* TODO: check range of vlan group */

	reg = VLAN_RW_VALID_5358;
	reg|= vlan_group << 12;
	reg|= B53_SHIFT(members, VLAN_RW_MEMBER);
	reg|= B53_SHIFT(untagged, VLAN_RW_UNTAGGED);
	error = b53chip_write4(sc, VLAN_WRITE, reg);
	if (error) {
		device_printf(sc->sc_dev, "can't write to VLAN_WRITE"
		    "[%d]: %d\n", vlan_group, error);
		return (error);
	}
	/* Set VLAN_GROUP as input */
	reg = VLAN_TABLE_ACCESS_RW_ENABLE;
	reg|= VLAN_TABLE_ACCESS_WRITE;
	reg|= vlan_group & VLAN_TABLE_ACCESS_VID_MASK;
	error = b53chip_write4(sc, VLAN_TABLE_ACCESS, reg);
	if (error) {
		device_printf(sc->sc_dev, "can't write to VLAN_TABLE_ACCESS"
		    "[%d]: %d\n", vlan_group, error);
		return (error);
	}

	return (0);
}

