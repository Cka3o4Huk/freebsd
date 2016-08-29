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
static int 	b53115hal_vlan_enable(struct b53_softc *sc, int on);

struct b53_functions b53115_f = {
	.reset = b53115hal_reset,
	.vlan_enable = b53115hal_vlan_enable,
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
b53115hal_vlan_enable(struct b53_softc *sc, int on)
{
	uint32_t	ctl0, ctl1, drop, ctl4, ctl5, sw;

	drop = 0;

	B53_RD(VLAN_GLOBAL_CTL0, ctl0, sc);
	B53_RD(VLAN_GLOBAL_CTL1, ctl1, sc);
	/* b53115-specific */
	B53_RD(VLAN_GLOBAL_CTL4_531xx, ctl4, sc);
	ctl4 &= ~VLAN_GLOBAL_CTL4_VID_MASK;
	B53_RD(VLAN_GLOBAL_CTL5_531xx, ctl5, sc);

	device_printf(sc->sc_dev, "ctl <=: %x/%x/%x/%x\n", ctl0, ctl1, ctl4,
	    ctl5);

	if (on) {
		ctl0 |= VLAN_GLOBAL_CTL0_1Q_ENABLE |
			    VLAN_GLOBAL_CTL0_MATCH_VIDMAC |
			    VLAN_GLOBAL_CTL0_HASH_VIDADDR;
		ctl1 |= VLAN_GLOBAL_CTL1_MCAST_UNTAGMAP_CHECK |
			    VLAN_GLOBAL_CTL1_MCAST_FWDMAP_CHECK;
		ctl4 |= VLAN_GLOBAL_CTL4_DROP_VID_VIOLATION;
		ctl5 |= VLAN_GLOBAL_CTL5_DROP_VTAB_MISS;

		/* b53115-specific: TODO: add support of 4095 vlans */
		ctl5 &= ~VLAN_GLOBAL_CTL5_VID_4095_ENABLE;
	} else {
		ctl0 &= ~(VLAN_GLOBAL_CTL0_1Q_ENABLE |
			    VLAN_GLOBAL_CTL0_MATCH_VIDMAC |
			    VLAN_GLOBAL_CTL0_HASH_VIDADDR);
		ctl1 &= ~(VLAN_GLOBAL_CTL1_MCAST_UNTAGMAP_CHECK |
			    VLAN_GLOBAL_CTL1_MCAST_FWDMAP_CHECK);
		ctl5 &= ~VLAN_GLOBAL_CTL5_DROP_VTAB_MISS;

		/* b53115-specific */
		ctl4 |= VLAN_GLOBAL_CTL4_FWD_TO_MII;
		ctl5 &= ~VLAN_GLOBAL_CTL5_VID_4095_ENABLE;
	}

	device_printf(sc->sc_dev, "ctl =>: %x/%x/%x/%x\n", ctl0, ctl1, ctl4,
	    ctl5);

	B53_WR(VLAN_GLOBAL_CTL0, ctl0, sc);
	B53_WR(VLAN_GLOBAL_CTL1, ctl1, sc);
	/* b53115-specific */
	B53_WR(VLAN_DROP_UNTAGGED, drop, sc);
	B53_WR(VLAN_GLOBAL_CTL4_531xx, ctl4, sc);
	B53_WR(VLAN_GLOBAL_CTL5_531xx, ctl5, sc);

	B53_RD(SWITCH_MODE, sw, sc);
	sw &= ~SWITCH_MODE_MANAGED;
	B53_WR(SWITCH_MODE, sw, sc);
	return (0);
}

static int
b53115hal_get_vlan_group(struct b53_softc *sc, int vlan_group,
		    int *vlan_id, int *members, int *untagged, int *forward_id)
{
	uint32_t		 reg;

	/* Set VLAN_GROUP as input */
	B53_WR(VLAN_TABLE_INDX_5395, vlan_group, sc);
	/* Execute READ vlan information */
	reg = VLAN_TABLE_ACCESS_5395_RUN | VLAN_TABLE_ACCESS_5395_READ;
	B53_WR(VLAN_TABLE_ACCESS_5395, reg, sc);
	/* Read result */
	B53_RD(VLAN_TABLE_ENTRY_5395, reg, sc);

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

	/* TODO: check range of vlan group */

	/* Set new entry */
	reg = B53_SHIFT(untagged, VLAN_RW_UNTAGGED_5395);
	reg |= B53_SHIFT(members, VLAN_RW_MEMBER_5395);
	B53_WR(VLAN_TABLE_ENTRY_5395, reg, sc);
	/* Set VLAN_GROUP as input */
	B53_WR(VLAN_TABLE_INDX_5395, vlan_group, sc);
	/* Execute READ vlan information */
	reg = VLAN_TABLE_ACCESS_5395_RUN;
	B53_WR(VLAN_TABLE_ACCESS_5395, reg, sc);

	return (0);
}

