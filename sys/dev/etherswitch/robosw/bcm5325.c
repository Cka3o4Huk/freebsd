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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * Ported version of bcm5325_switch from ZRouter.org
 * BCM5325 is 6-port managed 10/100 ROBOswitch with
 * port-based and .1q-based VLAN with 16 entries
 *
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/errno.h>
#include <sys/bus.h>

#include "robosw_var.h"
#include "robosw_reg.h"

#define	BCM5325_PBVLAN_PHYPORT_MASK	0x01F
#define	BCM5325_PBVLAN_MIIPORT_MASK	0x100

static int	bcm5325_reset(struct robosw_softc *sc);
static void	bcm5325_init(struct robosw_softc *sc);
static int	bcm5325_get_port_pvid(struct robosw_softc *sc, int port, int *pvid);
static int	bcm5325_set_port_pvid(struct robosw_softc *sc, int port, int pvid);
static int	bcm5325_vlan_enable_1q(struct robosw_softc *sc, int on);
static int	bcm5325_vlan_get_vlan_group(struct robosw_softc *sc, int vlan_group,
		    int *vlan_id, int *members, int *untagged, int *forward_id);
static int	bcm5325_vlan_set_vlan_group(struct robosw_softc *sc, int vlan_group,
		    int vlan_id, int members, int untagged, int forward_id);

static int	bcm5325_vlan_get_pvlan_group(struct robosw_softc *sc, int vlan_group,
		    int *vlan_id, int *members, int *untagged, int *forward_id);
static int	bcm5325_vlan_set_pvlan_group(struct robosw_softc *sc, int vlan_group,
		    int vlan_id, int members, int untagged, int forward_id);
static int	bcm5325_arl_iterator(struct robosw_softc *sc);
static int	bcm5325_arl_next(struct robosw_softc *sc, int *portmask,
		    uint8_t (*es_macaddr)[ETHER_ADDR_LEN], uint32_t *vid);

static int	bcm5325_arl_cleanup(struct robosw_softc *sc,
		    uint8_t (*es_macaddr)[ETHER_ADDR_LEN], uint32_t vid);
static int	bcm5325_arl_flushall(struct robosw_softc *sc);

static uint32_t	bcm5325_mib_get(struct robosw_softc *sc, int port, int metric);

struct robosw_functions bcm5325_f = {
	.api.reset = bcm5325_reset,
	.api.init_context = bcm5325_init,
	.api.mib_get = bcm5325_mib_get,
	.api.vlan_get_pvid = bcm5325_get_port_pvid,
	.api.vlan_set_pvid = bcm5325_set_port_pvid,
	.api.vlan_enable_1q = bcm5325_vlan_enable_1q,
	.api.vlan_get_vlan_group = bcm5325_vlan_get_vlan_group,
	.api.vlan_set_vlan_group = bcm5325_vlan_set_vlan_group,
	.api.vlan_get_pvlan_group = bcm5325_vlan_get_pvlan_group,
	.api.vlan_set_pvlan_group = bcm5325_vlan_set_pvlan_group,
	.api.arl_iterator = bcm5325_arl_iterator,
	.api.arl_next = bcm5325_arl_next,
	.api.arl_flushall = bcm5325_arl_flushall,
};

struct robosw_hal bcm5325_hal = {
	.chipname = "BCM5325",
	.parent = NULL,
	.self = &bcm5325_f
};

static void
bcm5325_init(struct robosw_softc *sc)
{

	/* BCM5325 doesn't support software reset */
	sc->sc_full_reset = ROBOSW_NORESET;
}

static uint32_t
bcm5325_mib_get(struct robosw_softc *sc, int port, int metric)
{
	switch (metric) {
	case ROBOSW_MIB_GOODRXPKTS:
		return robosw_read4(sc, MIB_5325_RX_GoodPkts(port));
	case ROBOSW_MIB_GOODTXPKTS:
		return robosw_read4(sc, MIB_5325_TX_GoodPkts(port));
	default:
		return (0);
	}
}

static int
bcm5325_reset(struct robosw_softc *sc)
{
	int		err;
	uint32_t	reg;

	/* MII port state override (page 0 register 14) */
	err = robosw_op(sc, PORTMII_STATUS_OVERRIDE, &reg, 0);

	if (err) {
		device_printf(sc->sc_dev, "Unable to set RvMII mode\n");
		return (ENXIO);
	}

	/* MIB provides 2 groups for choice - use group 0 for all ports */
	robosw_write4(sc, RMON_MIB_STEERING, 0);

	/* Bit 4 enables reverse MII mode */
	if (!(reg & PORTMII_STATUS_REVERSE_MII))
	{
		/* Enable RvMII */
		reg |= PORTMII_STATUS_REVERSE_MII;
		robosw_write4(sc, PORTMII_STATUS_OVERRIDE, reg);
		/* Read back */
		err = robosw_op(sc, PORTMII_STATUS_OVERRIDE, &reg, 0);
		if (err || !(reg & PORTMII_STATUS_REVERSE_MII))
		{
			device_printf(sc->sc_dev, "Unable to set RvMII mode: %x\n", reg);
			device_printf(sc->sc_dev, "or this chip doesn't support it\n");
			/* return (ENXIO); */
		}
	}

	/* Let's flush ARL */
	bcm5325_arl_flushall(sc);

	return (0);
}

static int
bcm5325_get_port_pvid(struct robosw_softc *sc, int port, int *pvid)
{
	int	local_pvid;

	local_pvid = robosw_read4(sc, VLAN_DEFAULT_PORT_TAG(port));
	if (local_pvid < 0)
		return (EBUSY);

	*pvid = local_pvid & 0xfff; /* TODO: define macro for mask */

	return (0);
}

static int
bcm5325_set_port_pvid(struct robosw_softc *sc, int port, int pvid)
{

	if (port > (sc->info.es_nports - 1))
		return (EINVAL);

	if (pvid > 0xfff)
		return (EINVAL);

	return (robosw_write4(sc, VLAN_DEFAULT_PORT_TAG(port), pvid));
}

static int
bcm5325_vlan_enable_1q(struct robosw_softc *sc, int on)
{
	uint32_t	ctl0, ctl1, drop, ctl4, ctl5, sw, mask_all_ports;
	int		i;

	drop = 0;

	ROBOSW_RD(VLAN_GLOBAL_CTL0, ctl0, sc);
	ROBOSW_RD(VLAN_GLOBAL_CTL1, ctl1, sc);
	ROBOSW_RD(VLAN_GLOBAL_CTL4, ctl4, sc);
	ROBOSW_RD(VLAN_GLOBAL_CTL5, ctl5, sc);

	if (sc->sc_debug)
		device_printf(sc->sc_dev, "ctl <=: %x/%x/%x/%x\n", ctl0, ctl1,
		    ctl4, ctl5);

	ctl0 |= VLAN_GLOBAL_CTL0_MATCH_VIDMAC |
		    VLAN_GLOBAL_CTL0_HASH_VIDADDR;

	mask_all_ports = (1 << sc->info.es_nports) - 1;
	i = 0;
	for (; i < sc->info.es_nports; i++)
		bcm5325_vlan_set_pvlan_group(sc, i, i, mask_all_ports, 0, 0);

	if (on) {
		/*
		 * CTL0: enable 1q and IVL (individual VLAN learning mode)
		 * CTL1:
		 */
		ctl0 |= VLAN_GLOBAL_CTL0_1Q_ENABLE;
		ctl1 |= VLAN_GLOBAL_CTL1_MCAST_TAGGING |
			VLAN_GLOBAL_CTL1_MCAST_UNTAGMAP_CHECK |
			    VLAN_GLOBAL_CTL1_MCAST_FWDMAP_CHECK;
		drop = 0xf0;
		ctl4 |= VLAN_GLOBAL_CTL4_DROP_VID_VIOLATION;
		ctl5 |= VLAN_GLOBAL_CTL5_APPLY_VLAN_FORWARD |
		    VLAN_GLOBAL_CTL5_DROP_VTAB_MISS;
	} else {
		ctl0 &= ~(VLAN_GLOBAL_CTL0_1Q_ENABLE);
		ctl1 &= ~(VLAN_GLOBAL_CTL1_MCAST_UNTAGMAP_CHECK |
			    VLAN_GLOBAL_CTL1_MCAST_FWDMAP_CHECK);
		drop = 0;
		ctl4 &= ~VLAN_GLOBAL_CTL4_DROP_VID_VIOLATION;
		ctl5 &= ~VLAN_GLOBAL_CTL5_APPLY_VLAN_FORWARD;
		ctl5 &= ~VLAN_GLOBAL_CTL5_DROP_VTAB_MISS;
	}

	if (sc->sc_debug)
		device_printf(sc->sc_dev, "ctl =>: %x/%x/%x/%x\n", ctl0, ctl1,
		    ctl4, ctl5);

	ROBOSW_WR(VLAN_GLOBAL_CTL0, ctl0, sc);
	ROBOSW_WR(VLAN_GLOBAL_CTL1, ctl1, sc);
	ROBOSW_WR(VLAN_DROP_UNTAGGED, drop, sc);
	ROBOSW_WR(VLAN_GLOBAL_CTL4, ctl4, sc);
	ROBOSW_WR(VLAN_GLOBAL_CTL5, ctl5, sc);

	ROBOSW_RD(SWITCH_MODE, sw, sc);
	sw &= ~SWITCH_MODE_MANAGED;
	ROBOSW_WR(SWITCH_MODE, sw, sc);
	return (0);
}

static int
bcm5325_vlan_get_vlan_group(struct robosw_softc *sc, int vlan_group,
    int *vlan_id, int *members, int *untagged, int *forward_id)
{
	uint32_t		 reg;

	/* Execute READ vlan information */
	reg = VLAN_TABLE_ACCESS_RW_ENABLE | (vlan_group & VLAN_TABLE_ACCESS_VID_MASK);
	ROBOSW_WR(VLAN_TABLE_ACCESS, reg, sc);
	/* Read result */
	ROBOSW_RD(VLAN_READ, reg, sc);

	if (sc->sc_debug)
		printf("read: dot1q[%d] = 0x%x\n", vlan_group, reg);

	*vlan_id = vlan_group | (ROBOSW_UNSHIFT(reg, VLAN_READ_HIGHVID) << 4) ;

	/*
	 * ETHERSWITCH_VID_VALID:
	 *   etherswitchcfg expects flag what VLAN is valid or not
	 */
	if (reg & VLAN_RW_VALID)
		*vlan_id |= ETHERSWITCH_VID_VALID;

	*members = ROBOSW_UNSHIFT(reg, VLAN_RW_MEMBER);
	*untagged = ROBOSW_UNSHIFT(reg, VLAN_RW_UNTAGGED);
	/* TODO: forwarding */
	*forward_id = 0; // RTL8366RB_VMCR_FID(vmcr);

	return (0);
}

static int
bcm5325_vlan_set_vlan_group(struct robosw_softc *sc, int vlan_group,
    int vlan_id, int members, int untagged, int forward_id)
{
	uint32_t		 reg;

	reg = ROBOSW_SHIFT(members, VLAN_RW_MEMBER) |
	    ROBOSW_SHIFT(untagged, VLAN_RW_UNTAGGED);

	reg |= VLAN_RW_VALID;

	if (sc->sc_debug)
		printf("write: dot1q[%d / 0x%x] = 0x%x\n", vlan_group, vlan_id, reg);

	ROBOSW_WR(VLAN_WRITE, reg, sc);

	reg = VLAN_TABLE_ACCESS_RW_ENABLE | VLAN_TABLE_ACCESS_WRITE |
	    (vlan_id & ETHERSWITCH_VID_MASK);
	ROBOSW_WR(VLAN_TABLE_ACCESS, reg, sc);

	return (0);
}

static int
bcm5325_vlan_get_pvlan_group(struct robosw_softc *sc, int vlan_group,
		    int *vlan_id, int *members, int *untagged, int *forward_id)
{
	uint32_t	 value;
	int		 reg;

	reg = (vlan_group & ETHERSWITCH_VID_MASK);

	/* Read port-based VLAN register */
	ROBOSW_RD(PBVLAN_ALLOWED_PORTS(reg), value, sc);

	if (sc->sc_debug)
		printf("read: pbvlan[%d] = 0x%x\n", vlan_group, value);

	/* Remap port MII -> last port */
	*vlan_id = (reg == sc->cpuport) ? (sc->info.es_nports - 1) : reg;

	/* etherswitchcfg expects flag ETHERSWITCH_VID_VALID what VLAN is valid */
	if (value & PBVLAN_ALLOWED_PORTS_MASK)
		*vlan_id |= ETHERSWITCH_VID_VALID;

	/* MII port uses bit 8*/
	*members = (value & BCM5325_PBVLAN_PHYPORT_MASK);
	if (value & BCM5325_PBVLAN_MIIPORT_MASK)
		*members |= (1 << (sc->info.es_nports - 1));

	*untagged = *members;

	return (0);
}

static int
bcm5325_vlan_set_pvlan_group(struct robosw_softc *sc, int vlan_group,
		    int vlan_id, int members, int untagged, int forward_id)
{
	uint32_t	 value;
	int		 reg;

	reg = (vlan_group & ETHERSWITCH_VID_MASK);
	ROBOSW_RD(PBVLAN_ALLOWED_PORTS(reg), value, sc);

	/* We ignore vlan_id redefinition for port-based VLANs */
	value &= ~(BCM5325_PBVLAN_PHYPORT_MASK | BCM5325_PBVLAN_MIIPORT_MASK);
	value |= (members & BCM5325_PBVLAN_PHYPORT_MASK);
	if (members & (1 << (sc->info.es_nports - 1)))
		value |= BCM5325_PBVLAN_MIIPORT_MASK;

	if (sc->sc_debug)
		printf("write: pbvlan[%d] = 0x%x\n", vlan_group, value);

	/* Read port-based VLAN register */
	ROBOSW_WR(PBVLAN_ALLOWED_PORTS(reg), value, sc);

	return (0);
}

/* ARL functions */
static int
bcm5325_arl_iterator(struct robosw_softc *sc)
{
	uint32_t	val;

	val = ARL_SEARCH_CTL_START;

	CTR1(KTR_DEV,"robosw: start ARL iterator: %x", val);
	ROBOSW_WR(ARL_SEARCH_CTL, val, sc);
	return (0);
}

static int
bcm5325_arl_next(struct robosw_softc *sc, int *portmask,
    uint8_t (*es_macaddr)[ETHER_ADDR_LEN], uint32_t *vid)
{
	uint32_t	val;
	uint64_t	res;
	int		ind;

	ind = ROBOSW_OP_RETRY;
	for (; ind > 0; ind--) {
		ROBOSW_RD(ARL_SEARCH_CTL, val, sc);
		if ((val & ARL_SEARCH_CTL_VALID) == ARL_SEARCH_CTL_VALID)
			break;

		if ((val & ARL_SEARCH_CTL_DONE) == 0) {
			CTR1(KTR_DEV,"robosw: finish ARL iterator: %x", val);
			return (ENOENT);
		}
	}

	ROBOSW_RD64(ARL_SEARCH_RESULT, res, sc);
	CTR3(KTR_DEV,"robosw: valid ARL: 0x%x 0x%x%x", val,
	    (res >> 32) & UINT32_MAX, res & UINT32_MAX);
	if (portmask != NULL) {
		*portmask = ROBOSW_UNSHIFT(res,ARL_SEARCH_RESULT_PORTID);
		CTR1(KTR_DEV,"robosw: portmask: 0x%x", *portmask);
	}

	if (es_macaddr != NULL)
	{
		(*es_macaddr)[0] = ROBOSW_UNSHIFT(res,ARL_SEARCH_RESULT_MAC0) & UINT8_MAX;
		(*es_macaddr)[1] = ROBOSW_UNSHIFT(res,ARL_SEARCH_RESULT_MAC1) & UINT8_MAX;
		(*es_macaddr)[2] = ROBOSW_UNSHIFT(res,ARL_SEARCH_RESULT_MAC2) & UINT8_MAX;
		(*es_macaddr)[3] = ROBOSW_UNSHIFT(res,ARL_SEARCH_RESULT_MAC3) & UINT8_MAX;
		(*es_macaddr)[4] = ROBOSW_UNSHIFT(res,ARL_SEARCH_RESULT_MAC4) & UINT8_MAX;
		(*es_macaddr)[5] = ROBOSW_UNSHIFT(res,ARL_SEARCH_RESULT_MAC5) & UINT8_MAX;
		CTR6(KTR_DEV,"robosw: mac addr: %x:%x:%x:%x:%x:%x",
		    (*es_macaddr)[0], (*es_macaddr)[1], (*es_macaddr)[2],
		    (*es_macaddr)[2], (*es_macaddr)[4], (*es_macaddr)[5]);
	}

	if (vid != NULL)
	{
		*vid = ROBOSW_UNSHIFT(res,ARL_SEARCH_RESULT_VID);
		CTR1(KTR_DEV,"robosw: vid: 0x%x", *vid);
	}

	return (0);
}

static int
bcm5325_arl_cleanup(struct robosw_softc *sc,
    uint8_t (*es_macaddr)[ETHER_ADDR_LEN], uint32_t vid)
{
	uint32_t	tmp;
	uint64_t	tmp64;
	int		ind;

	tmp64 = 0;
	tmp64 = tmp64 | ROBOSW_SHIFT((uint64_t)(*es_macaddr)[0],MAC_ADDRESS_INDEX_MAC0);
	tmp64 = tmp64 | ROBOSW_SHIFT((uint64_t)(*es_macaddr)[1],MAC_ADDRESS_INDEX_MAC1);
	tmp64 = tmp64 | ROBOSW_SHIFT((uint64_t)(*es_macaddr)[2],MAC_ADDRESS_INDEX_MAC2);
	tmp64 = tmp64 | ROBOSW_SHIFT((uint64_t)(*es_macaddr)[3],MAC_ADDRESS_INDEX_MAC3);
	tmp64 = tmp64 | ROBOSW_SHIFT((uint64_t)(*es_macaddr)[4],MAC_ADDRESS_INDEX_MAC4);
	tmp64 = tmp64 | ROBOSW_SHIFT((uint64_t)(*es_macaddr)[5],MAC_ADDRESS_INDEX_MAC5);

	device_printf(sc->sc_dev,
	    "lookup ARL by MAC: 0x%llx and VID: 0x%x\n",
	    tmp64, vid);
	CTR3(KTR_DEV,"robosw: lookup ARL by MAC: 0x%x%x and VID: 0x%x",
	    (tmp64 >> 32) & UINT32_MAX, tmp64 & UINT32_MAX, vid);

	/* Read bucket from ARL table by MAC address and VID */
	ROBOSW_WR64(MAC_ADDRESS_INDEX, tmp64, sc);
	ROBOSW_WR(VID_TABLE_INDEX, vid, sc);
	tmp = ARL_RW_CTL_READ;
	ROBOSW_WR(ARL_RW_CTL, tmp, sc);
	tmp = ARL_RW_CTL_START | ARL_RW_CTL_READ;
	ROBOSW_WR(ARL_RW_CTL, tmp, sc);

	ind = ROBOSW_OP_RETRY;
	for (; ind > 0; ind--) {
		ROBOSW_RD(ARL_RW_CTL, tmp, sc);
		if ((tmp & ARL_RW_CTL_DONE) == 0)
			break;
	}

	/* Timeout */
	if (ind == 0)
		return (ENXIO);

	ROBOSW_RD64(ARL_ENTRY_0, tmp64, sc);
	CTR2(KTR_DEV,"robosw: found ARL0: 0x%x%x",
	    (tmp64 >> 32) & UINT32_MAX, tmp64 & UINT32_MAX);

	tmp64 = ((tmp64 << 2) >> 2);

	CTR2(KTR_DEV,"robosw: reset ARL0: 0x%x%x",
	    (tmp64 >> 32) & UINT32_MAX, tmp64 & UINT32_MAX);

	ROBOSW_WR64(ARL_ENTRY_0, tmp64, sc);

	ROBOSW_RD64(ARL_ENTRY_1, tmp64, sc);
	CTR2(KTR_DEV,"robosw: found ARL1: 0x%x%x",
	    (tmp64 >> 32) & UINT32_MAX, tmp64 & UINT32_MAX);

	tmp64 = ((tmp64 << 2) >> 2);

	CTR2(KTR_DEV,"robosw: reset ARL1: 0x%x%x",
	    (tmp64 >> 32) & UINT32_MAX, tmp64 & UINT32_MAX);

	ROBOSW_WR64(ARL_ENTRY_1, tmp64, sc);

	/* Write changes */
	tmp = 0;
	ROBOSW_WR(ARL_RW_CTL, tmp, sc);
	tmp = ARL_RW_CTL_START;
	ROBOSW_WR(ARL_RW_CTL, tmp, sc);

	ind = ROBOSW_OP_RETRY;
	for (; ind > 0; ind--) {
		ROBOSW_RD(ARL_RW_CTL, tmp, sc);
		if ((tmp & ARL_RW_CTL_DONE) == 0)
			break;
	}

	/* Timeout */
	if (ind == 0)
		return (ENXIO);

	return (0);
}

static int
bcm5325_arl_flushall(struct robosw_softc *sc)
{
	int			i, err;
	uint8_t 		es_macaddr[ETHER_ADDR_LEN];
	uint32_t		vid;
	struct robosw_api	*api;

	i = ROBOSW_OP_RETRY;
	api = &(sc->hal.api);

	err = api->arl_iterator(sc);
	if (err != 0)
		return (err);

	/* finite cycle is used to avoid hang in case of hardware mistakes */
	for (;i>0;i--) {
		device_printf(sc->sc_dev, "looking for entry\n");
		err = api->arl_next(sc, NULL, &es_macaddr, &vid);
		if (err == ENOENT)
			break;
		else if (err != 0)
			return (err);
		device_printf(sc->sc_dev, "cleanup entry\n");
		err = bcm5325_arl_cleanup(sc, &es_macaddr, vid);
		if (err != 0)
			return (err);
	}

	return (0);
}
