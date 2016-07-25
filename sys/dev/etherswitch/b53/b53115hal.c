/*-
 * Copyright (c) 2011 Aleksandr Rybalko.
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
 *
 * $FreeBSD$
 */

#include <sys/types.h>
#include <sys/bus.h>
#include <sys/errno.h>

#include "b53_var.h"
#include "b53_reg.h"
#include "b53_hal.h"

static int	b53115hal_get_vlan_group(struct b53_softc *sc, int vlan_group,
		    int *vlan_id, int *members, int *untagged, int *forward_id);
static int	b53115hal_set_vlan_group(struct b53_softc *sc, int vlan_group,
		    int vlan_id, int members, int untagged, int forward_id);
static int 	b53115hal_reset(struct b53_softc *sc);

struct b53_functions b53115_f = {
	.reset = b53115hal_reset,
	.vlan_get_vlan_group = b53115hal_get_vlan_group,
	.vlan_set_vlan_group = b53115hal_set_vlan_group
};

struct b53_hal b53115_hal = {
	.parent = &b5325_hal,
	.own = &b53115_f
};

static int
b53115hal_reset(struct b53_softc *sc)
{
	int		err;
	uint32_t	reg;

	/* MII port state override (page 0 register 14) */
	err = b53chip_op(sc, PORTMII_STATUS_OVERRIDE, &reg, 0);

	if (err) {
		device_printf(sc->sc_dev, "Unable to set RvMII mode\n");
		return (ENXIO);
	}

	b53chip_write4(sc, PORTMII_STATUS_OVERRIDE, reg |
	    PORTMII_STATUS_FORCE_1000M | PORTMII_STATUS_FORCE_LINK );

	return (0);
}

static int
b53115hal_get_vlan_group(struct b53_softc *sc, int vlan_group,
		    int *vlan_id, int *members, int *untagged, int *forward_id)
{
	uint32_t		 reg;
	int			 error;

	/* Set VLAN_GROUP as input */
	error = b53chip_write4(sc, VLAN_TABLE_INDX_5395, vlan_group);
	if (error) {
		device_printf(sc->sc_dev, "can't write to VLAN_TABLE_INDX_5395:"
		    "%d\n", error);
		return (error);
	}

	/* Execute READ vlan information */
	reg = VLAN_TABLE_ACCESS_5395_RUN | VLAN_TABLE_ACCESS_5395_READ;
	error = b53chip_write4(sc, VLAN_TABLE_ACCESS_5395, reg);
	if (error) {
		device_printf(sc->sc_dev, "can't write to VLAN_TABLE_ACCESS_5395 "
		    "%d\n", error);
		return (error);
	}

	/* Read result */
	error = b53chip_op(sc, VLAN_TABLE_ENTRY_5395, &reg, 0);
	if (error){
		device_printf(sc->sc_dev, "can't read to VLAN_TABLE_ENTRY_5395 "
		"reg: %d\n", error);
		return (error);
	}
#if 0
	printf("reg[%d] = 0x%x\n",vlan_group, reg);
#endif

	*vlan_id = ETHERSWITCH_VID_VALID | vlan_group; /* XXX: ??? */
	*members = B53_UNSHIFT(reg, VLAN_RW_MEMBER_5395); /* TODO: shift MUST be 9 */
	*untagged = B53_UNSHIFT(reg, VLAN_RW_UNTAGGED_5395);
	/* TODO: forwarding */
	*forward_id = 0; // RTL8366RB_VMCR_FID(vmcr);

	return (0);
}

static int
b53115hal_set_vlan_group(struct b53_softc *sc, int vlan_group,
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

