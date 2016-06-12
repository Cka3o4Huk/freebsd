/*-
 * Copyright (c) 2011 Aleksandr Rybalko.
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
 */
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/errno.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
/* Required for etherswitch */
#include <sys/socket.h>
#include <sys/sockio.h>

#include <machine/bus.h>

#include "miibus_if.h"
#include "mdio_if.h"

/* Required for etherswitch */
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#include <net/bpf.h>

#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>

#include <dev/etherswitch/etherswitch.h>
#include <dev/mii/miivar.h>

#include "b53_var.h"
#include "b53_reg.h"
#include "b53_hal.h"

#include "etherswitch_if.h"

MALLOC_DEFINE(M_BCMSWITCH, "b53", "b53 etherswitch data structures");

/*
 * ********************* PROTOTYPES **********************************
 */

static int	b53_probe(device_t dev);
static int	b53_attach(device_t dev);

/* MII bus - delegate to parent */
static int	b53mii_writereg(device_t dev, int phy, int reg, int val);
static int	b53mii_readreg(device_t dev, int phy, int reg);

/* Polling of MII statuses */
static void	b53mii_tick(void *arg);
static void	b53mii_pollstat(struct b53_softc *sc);


/* Etherswitch interface */
static void	b53switch_lock(device_t dev);
static void	b53switch_unlock(device_t dev);
static int	b53switch_getvgroup(device_t dev, etherswitch_vlangroup_t *vg);
static int	b53switch_getport(device_t dev, etherswitch_port_t *p);
static int	b53switch_setport(device_t dev, etherswitch_port_t *p);

static etherswitch_info_t*	b53switch_getinfo(device_t dev);

/* HAL-specific functions */
static void	b53hal_init(struct b53_softc *sc, struct b53_hal* inithal);

/* Media callbacks */
static int	b53ifmedia_upd(struct ifnet *ifp);
static void	b53ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr);

/*********************** Implementation ************************************/

static int
b53_probe(device_t dev)
{
	/*
	 * TODO:
	 *  - init mtx
	 *  - fetch switch ID
	 *  - set description
	 */
	return (BUS_PROBE_DEFAULT);
}

static int
b53_attach(device_t dev)
{
	struct b53_softc	*sc;
	int			 i, err;
	char 			 name[IFNAMSIZ];
	uint32_t		 switchid;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_parent = device_get_parent(dev);

	if (sc->sc_parent == NULL)
		return (ENXIO);

	mtx_init(&sc->sc_mtx, "b53", NULL, MTX_DEF);

	if (b53chip_reset(dev))
		device_printf(dev, "reset failed\n");

	switchid = b53chip_read4(sc, SWITCH_DEVICEID);

	if(switchid == 0)
		/* assume integrated BCM5325 */
		switchid = 0x5325;

	device_printf(dev, "found switch BCM%x\n", switchid);

	/* TODO: identify hal by switch ID and chipcommon ID */
	b53hal_init(sc, &b5358_hal);

	b53chip_write4(sc, PORT_CTL(PORTMII), PORTMII_CTL_BCAST_ENABLED |
			PORTMII_CTL_MCAST_ENABLED | PORTMII_CTL_UCAST_ENABLED);

	if (bootverbose)
		B53DUMP();

	/*
	 * TODO:
	 *  - fetch etherswitch info
	 *  - create dummy ifnet
	 */
	sc->info.es_nports = B53_NUM_PHYS;
	strcpy(sc->info.es_name, "Broadcom 53xx switch");
	sc->info.es_nvlangroups = 16;
	sc->info.es_vlan_caps = 0;
	sc->info.es_vlan_caps = ETHERSWITCH_VLAN_DOT1Q | ETHERSWITCH_VLAN_PORT;

	for (i = 0; i < B53_NUM_PHYS - 1; i++) {
		sc->ifp[i] = if_alloc(IFT_ETHER);
		sc->ifp[i]->if_softc = sc;
		sc->ifp[i]->if_flags |= IFF_UP | IFF_BROADCAST | IFF_DRV_RUNNING
			| IFF_SIMPLEX;
		snprintf(name, IFNAMSIZ, "%sport", device_get_nameunit(dev));
		sc->ifname[i] = malloc(strlen(name)+1, M_DEVBUF, M_WAITOK);
		bcopy(name, sc->ifname[i], strlen(name)+1);
		if_initname(sc->ifp[i], sc->ifname[i], i);
		err = mii_attach(dev, &sc->miibus[i], sc->ifp[i],
			b53ifmedia_upd, b53ifmedia_sts,
			BMSR_DEFCAPMASK, i, MII_OFFSET_ANY, 0);

		if (err != 0) {
			device_printf(dev, "attaching PHY %d failed\n", i);
			/* TODO: cleanup */
			return (err);
		}
	}

	bus_generic_probe(dev);
	bus_generic_attach(dev);

	callout_init_mtx(&sc->callout_tick, &sc->sc_mtx, 0);

	B53_LOCK(sc);
	b53mii_tick(sc);
	B53_UNLOCK(sc);

	for(int i = 1; i < 4; i++) {
		struct etherswitch_vlangroup 	vg;

		vg.es_vlangroup = i;
		err = b53switch_getvgroup(dev, &vg);
		device_printf(dev, "[%d] err=%d; members=%x untagged=%x\n", i,
				err,
				vg.es_member_ports,
				vg.es_untagged_ports);
	}

	return (0);
}

/**************** MII interface - delegate to parents **********************/
static int
b53mii_readreg(device_t dev, int phy, int reg)
{

	return (MDIO_READREG(device_get_parent(dev), phy, reg));
}

static int
b53mii_writereg(device_t dev, int phy, int reg, int val)
{

	return (MDIO_WRITEREG(device_get_parent(dev), phy, reg, val));
}

static void
b53mii_tick(void *arg)
{
	struct b53_softc *sc = arg;

	b53mii_pollstat(sc);
	callout_reset(&sc->callout_tick, 100, b53mii_tick, sc);
}

static void
b53mii_pollstat(struct b53_softc *sc)
{

	for(int i = 0; i < sc->info.es_nports - 1; i++)
		mii_pollstat(device_get_softc(sc->miibus[i]));
}

static void
b53hal_init(struct b53_softc *sc, struct b53_hal* inithal)
{
	struct b53_functions	*scfunc;
	struct b53_functions	*initfunc;

	scfunc = &sc->hal;
	for (;;) {
		initfunc = inithal->own;

		/* inherit functions from initial HAL */
		for (int i = 0; i < B53HALSIZE; i++)
			if (scfunc->func[i] == NULL)
				scfunc->func[i] = initfunc->func[i];

		if (inithal->parent == NULL)
			break;

		/* has parent, next cycle */
		inithal = inithal->parent;
	}
}

/*************** SWITCH register access via PSEUDO PHY over MII ************/
int
b53chip_op(struct b53_softc *sc, uint32_t reg, uint32_t *res, int is_wr)
{
	uint64_t 	val;
	uint8_t		len, page;
	uint16_t	data_reg, data_val;
	int		i, tmp;

	if (res == NULL)
		return (EINVAL);

	val  = (is_wr) ? *res : 0;
	len  = (B53_UNSHIFT(reg, B53_LEN) + 1) / 2; /* in halfword */
	page =  B53_UNSHIFT(reg, B53_PAGE);
	reg  =  B53_UNSHIFT(reg, B53_REG);

	/* Set page & register ID into access/rw control */
	tmp = B53_SHIFT(page, ACCESS_CONTROL_PAGE) | ACCESS_CONTROL_RW;
	B53_WRITEREG(B53_ACCESS_CONTROL_REG, tmp);

	if (is_wr)
		for (i = 0; i < len; i++) {
			data_reg = B53_DATA_REG_BASE + i;
			data_val = (uint16_t)((val >> (16*i)) & 0xffff);
			B53_WRITEREG(data_reg, data_val);
		}

	tmp = B53_SHIFT(reg, RW_CONTROL_ADDR);
	tmp|= (is_wr) ? RW_CONTROL_WRITE : RW_CONTROL_READ;
	B53_WRITEREG(B53_RW_CONTROL_REG, tmp);

	/* is operation finished? */
	i = B53_OP_RETRY;
	for (; i > 0; i--)
		if ((B53_READREG(B53_RW_CONTROL_REG) &
		     RW_CONTROL_OP_MASK) == RW_CONTROL_NOP)
			break;

	/* timed out */
	if (i == 0) {
		device_printf(sc->sc_dev, "timeout\n");
		return (EBUSY);
	}

	/* get result */
	if (is_wr == 0) {
		for (i = 0; i < len; i++)
			val |= (B53_READREG(B53_DATA_REG_BASE + i)) << (16*i);
		*res = val & UINT32_MAX;
	}

	/* Check that read/write status should be NOP / idle */
	i = B53_READREG(B53_RW_STATUS_REG);
	if (i & RW_CONTROL_OP_MASK)
		device_printf(sc->sc_dev,
		    "RW operation not yet finished: reg=%08x status=%d\n",
		    reg, i);

	return (0);
}

uint32_t
b53chip_read4(struct b53_softc *sc, uint32_t reg)
{
	uint32_t	val;
	int		err;

	err = b53chip_op(sc, reg, &val, 0);
	if (err) {
		device_printf(sc->sc_dev, "read reg[0x%x] failed\n", reg);
		return (UINT32_MAX);
	}

	return (val);
}

int
b53chip_write4(struct b53_softc *sc, uint32_t reg, uint32_t val)
{
	uint32_t	tmp;
	int		err;

	tmp = val;
	err = b53chip_op(sc, reg, &tmp, 1);
	if (err)
		device_printf(sc->sc_dev, "write reg[0x%x] failed\n", reg);

	return (err);
}

int
b53chip_reset(device_t dev)
{
	struct b53_softc	*sc;
	int			 err;

	sc = device_get_softc(dev);

	err = b53chip_write4(sc, SWITCH_RESET, 0xffff);
	if (err) {
		device_printf(dev, "reset: can't set reset bits\n");
		return (err);
	}

	DELAY(10000);

	err = b53chip_write4(sc, SWITCH_RESET, 0x00);
	if (err)
		device_printf(dev, "reset: can't clears reset bits\n");

	return (err);
}

/****************** EtherSwitch interface **********************************/
static etherswitch_info_t*
b53switch_getinfo(device_t dev)
{
	struct b53_softc	*sc;

	sc = device_get_softc(dev);
	return &(sc->info);
}

static void
b53switch_lock(device_t dev)
{
        struct b53_softc *sc;

        sc = device_get_softc(dev);

        B53_LOCK_ASSERT(sc, SA_UNLOCKED);
        B53_LOCK(sc);
}

static void
b53switch_unlock(device_t dev)
{
        struct b53_softc *sc;

        sc = device_get_softc(dev);

        B53_LOCK_ASSERT(sc, SA_XLOCKED);
        B53_UNLOCK(sc);
}

static int
b53switch_getport(device_t dev, etherswitch_port_t *p)
{
	struct b53_softc	*sc;
	struct mii_data 	*mii;
	int			 err;

	sc = device_get_softc(dev);

	if (p->es_port >= sc->info.es_nports || p->es_port < 0)
		return (EINVAL);

	err = sc->hal.vlan_get_pvid(sc, p->es_port, &p->es_pvid);
	if (err)
		return (err);

	if(p->es_port != sc->info.es_nports - 1) {
		mii = device_get_softc(sc->miibus[p->es_port]);
		err = ifmedia_ioctl(mii->mii_ifp, &p->es_ifr, &mii->mii_media,
			  SIOCGIFMEDIA);
	} else {
		p->es_flags |= ETHERSWITCH_PORT_CPU;
		p->es_ifmr.ifm_status = IFM_ACTIVE | IFM_AVALID;
		p->es_ifmr.ifm_count = 0;
		p->es_ifmr.ifm_active = IFM_ETHER | IFM_100_TX | IFM_FDX;
		p->es_ifmr.ifm_current = IFM_ETHER | IFM_100_TX | IFM_FDX;
		p->es_ifmr.ifm_mask = 0;
	}

	return (0);
}

static int
b53switch_setport(device_t dev, etherswitch_port_t *p)
{
	struct b53_softc	*sc;
	struct ifmedia		*ifm;
	struct mii_data		*mii;
	int			 err;

	sc = device_get_softc(dev);
	if (p->es_port < 0 || p->es_port > sc->info.es_nports)
		return (EINVAL);

	err = sc->hal.vlan_set_pvid(sc, p->es_port, p->es_pvid);
	if (err)
		return (err);

	/* Do not allow media changes on CPU port. */
	if (p->es_port == sc->info.es_nports - 1)
		return (0);

	mii = device_get_softc(sc->miibus[p->es_port]);
	if (mii == NULL)
		return (ENXIO);

	ifm = &mii->mii_media;
	return (ifmedia_ioctl(mii->mii_ifp, &p->es_ifr, ifm, SIOCSIFMEDIA));
}

static int
b53switch_getvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct b53_softc	*sc;
	int			 err;

	sc = device_get_softc(dev);

	err = sc->hal.vlan_get_vlan_group(sc, vg->es_vlangroup,
			&vg->es_vid,
			&vg->es_member_ports,
			&vg->es_untagged_ports,
			&vg->es_fid);

	return (err);
}

static int
b53switch_setvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct b53_softc	*sc;
	int			 err;

	sc = device_get_softc(dev);

	err = sc->hal.vlan_set_vlan_group(sc, vg->es_vlangroup,
			vg->es_vid,
			vg->es_member_ports,
			vg->es_untagged_ports,
			vg->es_fid);
	return (err);
//	error = b53switch_write4(sc, VLAN_TABLE_INDX_5395, vid);
//	if (error) {
//		device_printf(dev, "can't write to VLAN_TABLE_INDX_5395 reg: %d\n",  error);
//		return (error);
//	}
//	error = b53switch_write4(sc, VLAN_TABLE_ACCESS_5395,
//	    VLAN_TABLE_ACCESS_5395_RUN |VLAN_TABLE_ACCESS_5395_READ);
//	if (error){
//		device_printf(dev, "can't write to VLAN_TABLE_ACCESS_5395 reg: %d\n",  error);
//		return (error);
//	}
//
//	error = b53switch_op(sc, VLAN_TABLE_ENTRY_5395, &reg, 0);
//	if (error){
//		device_printf(dev, "can't read to VLAN_TABLE_ENTRY_5395 reg: %d\n",  error);
//		return (error);
//	}

//	error = b53switch_write4(sc, VLAN_TABLE_INDX_5350, vid);
//	if (error) {
//		device_printf(dev, "can't write to VLAN_TABLE_INDX_5395 reg: %d\n",  error);
//		return (error);
//	}
//	error = b53switch_write4(sc, VLAN_TABLE_ACCESS_5350,
//	    VLAN_TABLE_ACCESS_5395_RUN |VLAN_TABLE_ACCESS_5395_READ);
//	if (error){
//		device_printf(dev, "can't write to VLAN_TABLE_ACCESS_5395 reg: %d\n",  error);
//		return (error);
//	}
//
//	error = b53switch_op(sc, VLAN_TABLE_ENTRY_5350, &reg, 0);
//	if (error){
//		device_printf(dev, "can't read to VLAN_TABLE_ENTRY_5395 reg: %d\n",  error);
//		return (error);
//	}
//
//	device_printf(dev, "REG = 0x%x\n", reg);
}

/****************** ifmedia callbacks **************************************/
static int
b53ifmedia_upd(struct ifnet *ifp)
{
	struct b53_softc	*sc;
	struct mii_data		*mii;
	device_t		 bus;

	sc = ifp->if_softc;
	bus = sc->miibus[ifp->if_dunit];
	mii = device_get_softc(bus);

	mii_mediachg(mii);
	return (0);
}

static void
b53ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct b53_softc	*sc;
	struct mii_data		*mii;
	device_t		 bus;

	sc = ifp->if_softc;
	bus = sc->miibus[ifp->if_dunit];
	mii = device_get_softc(bus);

	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	return;
}


static device_method_t b53_methods [] =
{
	DEVMETHOD(device_probe,		b53_probe),
	DEVMETHOD(device_attach,	b53_attach),

	/* bus interface */
	DEVMETHOD(bus_add_child,	device_add_child_ordered),

	/* mii interface */
	DEVMETHOD(miibus_readreg,	b53mii_readreg),
	DEVMETHOD(miibus_writereg,	b53mii_writereg),

	/* etherswitch interface */
	DEVMETHOD(etherswitch_getinfo,		b53switch_getinfo),
	DEVMETHOD(etherswitch_lock,		b53switch_lock),
	DEVMETHOD(etherswitch_unlock,		b53switch_unlock),
	DEVMETHOD(etherswitch_getport,		b53switch_getport),
	DEVMETHOD(etherswitch_setport,		b53switch_setport),
	DEVMETHOD(etherswitch_getvgroup,	b53switch_getvgroup),
	DEVMETHOD(etherswitch_setvgroup,	b53switch_setvgroup),
#if 0
	DEVMETHOD(etherswitch_setvgroup,	b53sxxhal_setvgroup),
	DEVMETHOD(etherswitch_readreg,		b53switch_readreg_wrapper),
	DEVMETHOD(etherswitch_writereg,		b53switch_writereg_wrapper),
	DEVMETHOD(etherswitch_readphyreg,	b53switch_readphy_wrapper),
	DEVMETHOD(etherswitch_writephyreg,	b53switch_writephy_wrapper),
	DEVMETHOD(etherswitch_getconf,		b53switch_getconf),
	DEVMETHOD(etherswitch_setconf,		b53switch_setconf),
#endif
	DEVMETHOD_END
};

static devclass_t b53_devclass;

DEFINE_CLASS_0(b53, b53_driver, b53_methods, sizeof(struct b53_softc));

DRIVER_MODULE(b53,         mdio, b53_driver, b53_devclass, 0, 0);
DRIVER_MODULE(etherswitch, b53, etherswitch_driver, etherswitch_devclass, 0, 0);
DRIVER_MODULE(miibus,      b53, miibus_driver, miibus_devclass, 0, 0);

MODULE_DEPEND(b53, mdio, 1, 1, 1);
