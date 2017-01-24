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

#include "robosw_var.h"
#include "robosw_reg.h"
#include "robosw_hal.h"

#include "etherswitch_if.h"

MALLOC_DEFINE(M_BCMSWITCH, "robosw", "robosw data structures");

/*
 * ********************* PROTOTYPES **********************************
 */

static int	robosw_probe(device_t dev);
static int	robosw_attach(device_t dev);

/* MII bus - delegate to parent */
static int	robosw_mii_writereg(device_t dev, int phy, int reg, int val);
static int	robosw_mii_readreg(device_t dev, int phy, int reg);

/* Polling of MII statuses */
static void	robosw_mii_tick(void *arg);
static void	robosw_mii_pollstat(struct robosw_softc *sc);

static etherswitch_info_t*	robosw_getinfo(device_t dev);

/* Media callbacks */
static int	robosw_ifm_upd(struct ifnet *ifp);
static void	robosw_ifm_sts(struct ifnet *ifp, struct ifmediareq *ifmr);

/*********************** Implementation ************************************/

static int
robosw_probe(device_t dev)
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
robosw_attach(device_t dev)
{
	struct robosw_softc	*sc;
	int			 i, err, is_robosw_reset;
	char 			 name[IFNAMSIZ];
#if 0
	uint32_t		 reg;
#endif

	is_robosw_reset = 1;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_parent = device_get_parent(dev);

	if (sc->sc_parent == NULL)
		return (ENXIO);

	mtx_init(&sc->sc_mtx, "robosw", NULL, MTX_DEF);

	sc->info.es_nports = 5; //B53_NUM_PHYS
	strcpy(sc->info.es_name, "Broadcom RoboSwitch");
	sc->info.es_nvlangroups = 16;
	sc->info.es_vlan_caps = 0;
	sc->info.es_vlan_caps = ETHERSWITCH_VLAN_DOT1Q | ETHERSWITCH_VLAN_PORT;

	/* Initialize HAL by switch ID and chipcommon ID */
	robosw_hal_init(sc);

	/* Check HAL prerequisites */
	if (sc->hal.api.vlan_enable == NULL) {
		/* TODO: if (err) add detach */
		device_printf(dev, "[ERROR] no HAL for enable 1q\n");
		goto fail;
	}

	/* Last step before chip operations */
	if (bootverbose)
		ROBOSWDUMP;

	/* Turn off forwarding - start manipulations with rules/routes */
	err = robosw_enable_fw(dev, 0);
	if (err != 0) {
		device_printf(dev, "can't disable forwarding\n");
		goto fail;
	}

	/*
	 * XXX: Avoid default configuration, bootloader must set it or we
	 * must load user defined
	 */
	if (is_robosw_reset & robosw_reset(dev))
		device_printf(dev, "reset failed\n");

	if (sc->hal.api.reset != NULL) {
		sc->hal.api.reset(sc);
		/* TODO: if (err) add detach */
	}

	/* Enable VLAN support */
	sc->hal.api.vlan_enable(sc, 1);
	sc->hal.api.vlan_set_vlan_group(sc, ROBOSW_DEF_VLANID, ROBOSW_DEF_VLANID,
	    ROBOSW_DEF_MASK, ROBOSW_DEF_MASK, 0);

	/* At startup, let's specify all physical ports as one default VLAN group */
	for (i = 0; i < sc->info.es_nports; i++) {
		int	ctl;
		/* set PVID to default value */
		sc->hal.api.vlan_set_pvid(sc, i, ROBOSW_DEF_VLANID);

		/* enable Tx/Rx on physical port */
		ctl = robosw_read4(sc, PORT_CTL(i));
		ctl &= ~PORT_CTL_STP_STATE_MASK;
		ctl &= ~PORT_CTL_TX_DISABLED & ~PORT_CTL_RX_DISABLED;
		robosw_write4(sc, PORT_CTL(i), ctl);
	}

	/* Allows broadcast, unicast and multicast on CPU port */
	robosw_write4(sc, PORT_CTL(PORTMII), PORTMII_CTL_BCAST_ENABLED |
		PORTMII_CTL_MCAST_ENABLED | PORTMII_CTL_UCAST_ENABLED);

#if 0
	/* PAUSE capability handling */
	if (reg & PORTMII_STATUS_PAUSE_CAPABLE)
	{
		reg = robosw_read4(sc, PORTMII_STATUS_OVERRIDE);
		/* Disable pause */
		reg &= ~((uint32_t)PORTMII_STATUS_PAUSE_CAPABLE);

		robosw_write4(sc, PORTMII_STATUS_OVERRIDE, reg);
		/* Read back */
		err = robosw_op(sc, PORTMII_STATUS_OVERRIDE, &reg, 0);
		if (err || !(reg & PORTMII_STATUS_PAUSE_CAPABLE))
		{
			device_printf(dev, "[ERROR] Unable to resume chip\n");
			/* TODO: add detach */
			/* return (ENXIO); */
		}
	}
#endif

	/* Turn on forwarding - done with manipulations with rules/routes */
	err = robosw_enable_fw(dev, 1);
	if (err != 0) {
		device_printf(dev, "can't enable forwarding\n");
		goto fail;
	}

	if (bootverbose)
		ROBOSWDUMP;

	/*
	 * TODO:
	 *  - fetch etherswitch info
	 *  - create dummy ifnet
	 */

	for (i = 0; i < sc->info.es_nports; i++) {
		sc->ifp[i] = if_alloc(IFT_ETHER);
		sc->ifp[i]->if_softc = sc;
		sc->ifp[i]->if_flags |= IFF_UP | IFF_BROADCAST | IFF_DRV_RUNNING
			| IFF_SIMPLEX;
		snprintf(name, IFNAMSIZ, "%sport", device_get_nameunit(dev));
		sc->ifname[i] = malloc(strlen(name)+1, M_DEVBUF, M_WAITOK);
		bcopy(name, sc->ifname[i], strlen(name)+1);
		if_initname(sc->ifp[i], sc->ifname[i], i);
		err = mii_attach(dev, &sc->miibus[i], sc->ifp[i],
			robosw_ifm_upd, robosw_ifm_sts,
			BMSR_DEFCAPMASK, i, MII_OFFSET_ANY, 0);

		if (err != 0) {
			device_printf(dev, "attaching PHY %d failed: %d\n", i,
			    err);
			goto fail;
		}
	}

	bus_generic_probe(dev);
	bus_generic_attach(dev);

	callout_init_mtx(&sc->callout_tick, &sc->sc_mtx, 0);

	ROBOSW_LOCK(sc);
	robosw_mii_tick(sc);
	ROBOSW_UNLOCK(sc);

	return (0);
fail:
	/* TODO: detach */
	return (ENXIO);
}

/**************** MII interface - delegate to parents **********************/
static int
robosw_mii_readreg(device_t dev, int phy, int reg)
{

	return (MDIO_READREG(device_get_parent(dev), phy, reg));
}

static int
robosw_mii_writereg(device_t dev, int phy, int reg, int val)
{

	return (MDIO_WRITEREG(device_get_parent(dev), phy, reg, val));
}

static void
robosw_mii_tick(void *arg)
{
	struct robosw_softc *sc = arg;

	robosw_mii_pollstat(sc);
	callout_reset(&sc->callout_tick, 100, robosw_mii_tick, sc);
}

static void
robosw_mii_pollstat(struct robosw_softc *sc)
{

	for(int i = 0; i < sc->info.es_nports - 1; i++)
		mii_pollstat(device_get_softc(sc->miibus[i]));
}

/*************** SWITCH register access via PSEUDO PHY over MII ************/
int
robosw_op(struct robosw_softc *sc, uint32_t reg, uint32_t *res, int is_write)
{
	uint64_t 	val;
	uint8_t		len, page;
	uint16_t	data_reg, data_val;
	int		i, tmp;

	if (res == NULL)
		return (EINVAL);

	val  = (is_write) ? *res : 0;
	len  = (ROBOSW_UNSHIFT(reg, ROBOSW_LEN) + 1) / 2; /* in halfword */
	page =  ROBOSW_UNSHIFT(reg, ROBOSW_PAGE);
	reg  =  ROBOSW_UNSHIFT(reg, ROBOSW_REG);

	/* Set page & register ID into access/rw control */
	tmp = ROBOSW_SHIFT(page, ACCESS_CONTROL_PAGE) | ACCESS_CONTROL_RW;
	ROBOSW_WRITEREG(ROBOSW_ACCESS_CONTROL_REG, tmp);

	if (is_write)
		for (i = 0; i < len; i++) {
			data_reg = ROBOSW_DATA_REG_BASE + i;
			data_val = (uint16_t)((val >> (16*i)) & 0xffff);
			ROBOSW_WRITEREG(data_reg, data_val);
		}

	tmp = ROBOSW_SHIFT(reg, RW_CONTROL_ADDR);
	tmp|= (is_write) ? RW_CONTROL_WRITE : RW_CONTROL_READ;
	ROBOSW_WRITEREG(ROBOSW_RW_CONTROL_REG, tmp);

	/* is operation finished? */
	i = ROBOSW_OP_RETRY;
	for (; i > 0; i--)
		if ((ROBOSW_READREG(ROBOSW_RW_CONTROL_REG) &
		     RW_CONTROL_OP_MASK) == RW_CONTROL_NOP)
			break;

	/* timed out */
	if (i == 0) {
		device_printf(sc->sc_dev, "timeout\n");
		return (EBUSY);
	}

	/* get result */
	if (is_write == 0) {
		for (i = 0; i < len; i++)
			val |= (ROBOSW_READREG(ROBOSW_DATA_REG_BASE + i)) << (16*i);
		*res = val & UINT32_MAX;
	}

	/* Check that read/write status should be NOP / idle */
	i = ROBOSW_READREG(ROBOSW_RW_STATUS_REG);
	if (i & RW_CONTROL_OP_MASK)
		device_printf(sc->sc_dev,
		    "RW operation not yet finished: reg=%08x status=%d\n",
		    reg, i);

	return (0);
}

uint32_t
robosw_read4(struct robosw_softc *sc, uint32_t reg)
{
	uint32_t	val;
	int		err;

	err = robosw_op(sc, reg, &val, 0);
	if (err) {
		device_printf(sc->sc_dev, "read reg[0x%x] failed\n", reg);
		return (UINT32_MAX);
	}

	return (val);
}

int
robosw_write4(struct robosw_softc *sc, uint32_t reg, uint32_t val)
{
	uint32_t	tmp;
	int		err;

	tmp = val;
	err = robosw_op(sc, reg, &tmp, 1);
	if (err)
		device_printf(sc->sc_dev, "write reg[0x%x] failed\n", reg);

	return (err);
}

int
robosw_reset(device_t dev)
{
	struct robosw_softc	*sc;
	int			 err;

	sc = device_get_softc(dev);

	err = robosw_write4(sc, SWITCH_RESET, 0xffff);
	if (err) {
		device_printf(dev, "reset: can't set reset bits\n");
		return (err);
	}

	DELAY(10000);

	err = robosw_write4(sc, SWITCH_RESET, 0x00);
	if (err)
		device_printf(dev, "reset: can't clears reset bits\n");

	return (err);
}

int
robosw_enable_fw(device_t dev, uint32_t forward)
{
	struct robosw_softc	*sc;
	uint32_t		 reg;
	int 			 err;

	sc = device_get_softc(dev);

	err = robosw_op(sc, SWITCH_MODE, &reg, 0);
	if (err) {
		device_printf(dev, "enable_fw: can't read SWITCH_MODE reg\n");
		return (err);
	}
	if (forward == 0)
		reg &= ~SWITCH_MODE_FORWARDING_ENABLED;
	else
		reg |= SWITCH_MODE_FORWARDING_ENABLED;

	err = robosw_op(sc, SWITCH_MODE, &reg, 1);
	if (err) {
		device_printf(dev, "enable_fw: can't set SWITCH_MODE reg\n");
		return (err);
	}

	return 0;
}

/****************** EtherSwitch interface **********************************/
static etherswitch_info_t*
robosw_getinfo(device_t dev)
{
	struct robosw_softc	*sc;

	sc = device_get_softc(dev);
	return &(sc->info);
}

void
robosw_lock(device_t dev)
{
        struct robosw_softc *sc;

        sc = device_get_softc(dev);

        ROBOSW_LOCK_ASSERT(sc, SA_UNLOCKED);
        ROBOSW_LOCK(sc);
}

void
robosw_unlock(device_t dev)
{
        struct robosw_softc *sc;

        sc = device_get_softc(dev);

        ROBOSW_LOCK_ASSERT(sc, SA_XLOCKED);
        ROBOSW_UNLOCK(sc);
}

int
robosw_getport(device_t dev, etherswitch_port_t *p)
{
	struct robosw_softc	*sc;
	struct mii_data 	*mii;
	int			 err;

	sc = device_get_softc(dev);

	if (p->es_port >= sc->info.es_nports || p->es_port < 0)
		return (EINVAL);

	err = sc->hal.api.vlan_get_pvid(sc, p->es_port, &p->es_pvid);
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

int
robosw_setport(device_t dev, etherswitch_port_t *p)
{
	struct robosw_softc	*sc;
	struct ifmedia		*ifm;
	struct mii_data		*mii;
	int			 err;

	sc = device_get_softc(dev);
	if (p->es_port < 0 || p->es_port > sc->info.es_nports)
		return (EINVAL);

	err = sc->hal.api.vlan_set_pvid(sc, p->es_port, p->es_pvid);
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

int
robosw_getvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct robosw_softc	*sc;
	int			 err;

	sc = device_get_softc(dev);

	if (sc->hal.api.vlan_get_vlan_group == NULL)
		return (ENXIO);

	err = sc->hal.api.vlan_get_vlan_group(sc, vg->es_vlangroup,
			&vg->es_vid,
			&vg->es_member_ports,
			&vg->es_untagged_ports,
			&vg->es_fid);

	return (err);
}

int
robosw_setvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct robosw_softc	*sc;
	int			 err;

	sc = device_get_softc(dev);

	if (sc->hal.api.vlan_set_vlan_group == NULL)
		return (ENXIO);

	err = sc->hal.api.vlan_set_vlan_group(sc, vg->es_vlangroup,
			vg->es_vid,
			vg->es_member_ports,
			vg->es_untagged_ports,
			vg->es_fid);
	return (err);

}

/****************** ifmedia callbacks **************************************/
static int
robosw_ifm_upd(struct ifnet *ifp)
{
	struct robosw_softc	*sc;
	struct mii_data		*mii;
	device_t		 bus;

	sc = ifp->if_softc;
	bus = sc->miibus[ifp->if_dunit];
	mii = device_get_softc(bus);

	mii_mediachg(mii);
	return (0);
}

static void
robosw_ifm_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct robosw_softc	*sc;
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


static device_method_t robosw_methods [] =
{
	DEVMETHOD(device_probe,		robosw_probe),
	DEVMETHOD(device_attach,	robosw_attach),

	/* bus interface */
	DEVMETHOD(bus_add_child,	device_add_child_ordered),

	/* mii interface */
	DEVMETHOD(miibus_readreg,	robosw_mii_readreg),
	DEVMETHOD(miibus_writereg,	robosw_mii_writereg),

	/* etherswitch interface */
	DEVMETHOD(etherswitch_getinfo,		robosw_getinfo),
	DEVMETHOD(etherswitch_lock,		robosw_lock),
	DEVMETHOD(etherswitch_unlock,		robosw_unlock),
	DEVMETHOD(etherswitch_getport,		robosw_getport),
	DEVMETHOD(etherswitch_setport,		robosw_setport),
	DEVMETHOD(etherswitch_getvgroup,	robosw_getvgroup),
	DEVMETHOD(etherswitch_setvgroup,	robosw_setvgroup),
#if 0
	DEVMETHOD(etherswitch_readreg,		robosw_readreg_wrapper),
	DEVMETHOD(etherswitch_writereg,		robosw_writereg_wrapper),
	DEVMETHOD(etherswitch_readphyreg,	robosw_readphy_wrapper),
	DEVMETHOD(etherswitch_writephyreg,	robosw_writephy_wrapper),
	DEVMETHOD(etherswitch_getconf,		robosw_getconf),
	DEVMETHOD(etherswitch_setconf,		robosw_setconf),
#endif
	DEVMETHOD_END
};

static devclass_t robosw_devclass;

DEFINE_CLASS_0(robosw, robosw_driver, robosw_methods, sizeof(struct robosw_softc));

DRIVER_MODULE(robosw,      mdio, robosw_driver, robosw_devclass, 0, 0);
DRIVER_MODULE(etherswitch, robosw, etherswitch_driver, etherswitch_devclass, 0, 0);
DRIVER_MODULE(miibus,      robosw, miibus_driver, miibus_devclass, 0, 0);

MODULE_DEPEND(robosw, mdio, 1, 1, 1);
