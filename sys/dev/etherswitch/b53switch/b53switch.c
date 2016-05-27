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
#include <sys/systm.h>
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

#include "b53switch.h"
#include "b53switchvar.h"
#include "b53switchreg.h"

#include "etherswitch_if.h"

/*
 * ********************* PROTOTYPES **********************************
 */

static int			b53switch_probe(device_t dev);
static int			b53switch_attach(device_t dev);

static void			b53switch_lock(device_t dev);
static void			b53switch_unlock(device_t dev);
static etherswitch_info_t*	b53switch_getinfo(device_t dev);

static int			b53switch_op(struct b53switch_softc *sc,
		  	  	    uint32_t reg, uint32_t *res, int is_wr);
static uint32_t			b53switch_read4(struct b53switch_softc *sc,
				    uint32_t reg);
static int			b53switch_write4(struct b53switch_softc *sc,
				    uint32_t reg, uint32_t val);

static int			b53switch_reset(device_t dev);

static int			b53switch_ifmedia_upd(struct ifnet *ifp);
static void			b53switch_ifmedia_sts(struct ifnet *ifp,
			    	    struct ifmediareq *ifmr);

static int	b53switch_writereg(device_t dev, int phy, int reg, int val);
static int	b53switch_readreg(device_t dev, int phy, int reg);

/* Polling of MII statuses */
static void			b53switch_tick(void *arg);
static void			b53switch_miipollstat(struct b53switch_softc *sc);

static int			b53switch_getvgroup(device_t dev,
				    etherswitch_vlangroup_t *vg);

static int
b53switch_probe(device_t dev)
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
b53switch_attach(device_t dev)
{
	struct b53switch_softc	*sc;
	int			 i, err;
	char 			 name[IFNAMSIZ];

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_parent = device_get_parent(dev);

	mtx_init(&sc->sc_mtx, "b53sw", NULL, MTX_DEF);

	if (sc->sc_parent == NULL)
		return (ENXIO);

	/*
	 * TODO:
	 *  - fetch etherswitch info
	 *  - create dummy ifnet
	 */
	sc->info.es_nports = B53SWITCH_NUM_PHYS;
	strcpy(sc->info.es_name, "Broadcom 53xx switch");
	sc->info.es_nvlangroups = 16;
	sc->info.es_vlan_caps = 0;

	for (i = 0; i < B53SWITCH_NUM_PHYS; i++) {
		sc->ifp[i] = if_alloc(IFT_ETHER);
		sc->ifp[i]->if_softc = sc;
		sc->ifp[i]->if_flags |= IFF_UP | IFF_BROADCAST | IFF_DRV_RUNNING
			| IFF_SIMPLEX;
		snprintf(name, IFNAMSIZ, "%sport", device_get_nameunit(dev));
		sc->ifname[i] = malloc(strlen(name)+1, M_DEVBUF, M_WAITOK);
		bcopy(name, sc->ifname[i], strlen(name)+1);
		if_initname(sc->ifp[i], sc->ifname[i], i);
		err = mii_attach(dev, &sc->miibus[i], sc->ifp[i],
			b53switch_ifmedia_upd, b53switch_ifmedia_sts,
			BMSR_DEFCAPMASK, i, MII_OFFSET_ANY, 0);
		if (err != 0) {
			device_printf(dev, "attaching PHY %d failed\n", i);
			return (err);
		}
	}

	if (b53switch_reset(dev))
		device_printf(dev, "reset failed\n");

#define DUMP(_reg) device_printf(dev, #_reg "=%08x\n", b53switch_read4(sc, _reg))
	DUMP(PORT_CTL(PORT0));
	DUMP(PORT_CTL(PORT1));
	DUMP(PORT_CTL(PORT2));
	DUMP(PORT_CTL(PORT3));
	DUMP(PORT_CTL(PORT4));
	DUMP(PORT_CTL(PORT5));
	DUMP(PORT_CTL(PORT6));
	DUMP(PORT_CTL(PORT7));
	DUMP(PORT_CTL(PORTMII));
	DUMP(PORTMII_STATUS_OVERRIDE);
	DUMP(SWITCH_DEVICEID);
	DUMP(SWITCH_MODE);
	DUMP(POWER_DOWN_MODE);
	DUMP(LINK_STATUS_SUMMARY);
	DUMP(BIST_STATUS_RC);
	DUMP(VLAN_GLOBAL_CTL0);
	DUMP(VLAN_GLOBAL_CTL1);
	DUMP(VLAN_GLOBAL_CTL2);
	DUMP(VLAN_DROP_UNTAGGED);
	DUMP(VLAN_GLOBAL_CTL4);
	DUMP(VLAN_GLOBAL_CTL5);
#undef DUMP

	bus_generic_probe(dev);
	bus_generic_attach(dev);

	callout_init_mtx(&sc->callout_tick, &sc->sc_mtx, 0);

	/* TODO: add locking */
	//ARSWITCH_LOCK(sc);
	b53switch_tick(sc);
	//ARSWITCH_UNLOCK(sc);


	for(int i = 0; i< 10; i++) {
		struct etherswitch_vlangroup vg;

		vg.es_vlangroup = i;
		b53switch_getvgroup(dev, &vg);
		printf("[%d] %x %x \n", i,
				vg.es_member_ports,
				vg.es_untagged_ports);
	}


	return (0);
}

static void
b53switch_tick(void *arg)
{
	struct b53switch_softc *sc = arg;

	b53switch_miipollstat(sc);
	callout_reset(&sc->callout_tick, 100, b53switch_tick, sc);
}

static void
b53switch_miipollstat(struct b53switch_softc *sc)
{
	struct mii_data		*mii;

	for(int i = 0; i < B53SWITCH_NUM_PHYS; i++) {
		mii = device_get_softc(sc->miibus[i]);
		mii_pollstat(mii);
	}
}

static etherswitch_info_t*
b53switch_getinfo(device_t dev)
{
	struct b53switch_softc *sc;

	sc = device_get_softc(dev);

	return &(sc->info);
}

static void
b53switch_lock(device_t dev)
{
	struct b53switch_softc *sc;

	sc = device_get_softc(dev);

	B53SWITCH_LOCK_ASSERT(sc, SA_UNLOCKED);
	B53SWITCH_LOCK(sc);
}

static void
b53switch_unlock(device_t dev)
{
	struct b53switch_softc *sc;

	sc = device_get_softc(dev);

	B53SWITCH_LOCK_ASSERT(sc, SA_XLOCKED);
	B53SWITCH_UNLOCK(sc);
}

static int
b53switch_ifmedia_upd(struct ifnet *ifp)
{
	struct b53switch_softc *sc = ifp->if_softc;
	struct mii_data *mii = device_get_softc(sc->miibus[ifp->if_dunit]);

	mii_mediachg(mii);
	return (0);
}

static void
b53switch_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct b53switch_softc	*sc;
	struct mii_data		*mii;
	device_t		 bus;

	sc = ifp->if_softc;
	bus = sc->miibus[ifp->if_dunit];
	mii = device_get_softc(bus);

	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
}

static int
b53switch_op(struct b53switch_softc *sc, uint32_t reg, uint32_t *res, int is_wr)
{
	uint64_t 	val;
	uint8_t		len, page;
	int		i, tmp;

	val  = (is_wr) ? *res : 0;
	len  = (B53_UNSHIFT(reg, B53_LEN) + 1) / 2; /* in halfword */
	page = B53_UNSHIFT(reg, B53_PAGE);
	reg  = B53_UNSHIFT(reg, B53_REG);

	/* Set page & register ID into access/rw control */
	tmp = B53_SHIFT(page, ACCESS_CONTROL_PAGE) | ACCESS_CONTROL_RW;
	B53_WRITEREG(B53_ACCESS_CONTROL_REG, tmp);

	if (is_wr)
		for (i = 0; i < len; i++)
			B53_WRITEREG(B53_DATA_15_0_REG + i,
			    (uint16_t)((val >> (16*i)) & 0xffff));

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
			val |= (B53_READREG(B53_DATA_15_0_REG + i)) << (16*i);
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



static int
b53switch_readreg(device_t dev, int phy, int reg)
{
	return (MDIO_READREG(device_get_parent(dev), phy, reg));
}

static int
b53switch_writereg(device_t dev, int phy, int reg, int val)
{
	return (MDIO_WRITEREG(device_get_parent(dev), phy, reg, val));
}


static uint32_t
b53switch_read4(struct b53switch_softc *sc, uint32_t reg)
{
	uint32_t	val;
	int		err;

	err = b53switch_op(sc, reg, &val, 0);
	if (err) {
		device_printf(sc->sc_dev, "read reg[0x%x] failed\n", reg);
		return (UINT32_MAX);
	}

	return (val);
}

static int
b53switch_write4(struct b53switch_softc *sc, uint32_t reg, uint32_t val)
{
	uint32_t	tmp;
	int		err;

	tmp = val;
	err = b53switch_op(sc, reg, &tmp, 1);
	if (err)
		device_printf(sc->sc_dev, "write reg[0x%x] failed\n", reg);

	return (err);
}

static int
b53switch_reset(device_t dev)
{
	struct b53switch_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	error = b53switch_write4(sc, SWITCH_RESET, 0xffff);
	if (error) {
		device_printf(dev, "reset: can't set reset bits\n");
		return (error);
	}

	DELAY(10000);

	error = b53switch_write4(sc, SWITCH_RESET, 0x00);
	if (error)
		device_printf(dev, "reset: can't clears reset bits\n");

	return (error);
}

static int
b53switch_getport(device_t dev, etherswitch_port_t *p)
{
	struct b53switch_softc	*sc;
	struct mii_data 	*mii;
//	struct ifmediareq	*ifmr;
	int			 err;

	err = 0;

	sc = device_get_softc(dev);

//	B53SWITCH_LOCK_ASSERT(sc, SA_UNLOCKED);
//	B53SWITCH_LOCK(sc);

	if (p->es_port >= B53SWITCH_NUM_PHYS ||
	    p->es_port < 0) {
		err = EINVAL;
		goto out;
	}

	p->es_pvid = b53switch_read4(sc, VLAN_DEFAULT_PORT_TAG(p->es_port));

	if (p->es_pvid < 0) {
		err = EBUSY;
		goto out;
	}

	p->es_pvid &= 0xfff; /* TODO: define macro for mask */

	/* TODO: add CPU port support (can't find it on 5358U */
	mii = device_get_softc(sc->miibus[p->es_port]);
	err = ifmedia_ioctl(mii->mii_ifp, &p->es_ifr, &mii->mii_media,
		  SIOCGIFMEDIA);
//	if (e6000sw_cpuport(sc, p->es_port)) {
//		p->es_flags |= ETHERSWITCH_PORT_CPU;
//		ifmr = &p->es_ifmr;
//		ifmr->ifm_status = IFM_ACTIVE | IFM_AVALID;
//		ifmr->ifm_count = 0;
//		ifmr->ifm_current = ifmr->ifm_active =
//		    IFM_ETHER | IFM_100_T | IFM_FDX;
//		ifmr->ifm_mask = 0;
//	} else {
//	}

out:
//	B53SWITCH_UNLOCK(sc);
	return (err);
}

#if 0
static int	bcm5325_switch_probe(device_t dev);
static int	bcm5325_switch_attach(device_t dev);
static int	bcm5325_switch_detach(device_t dev);

/* TODO */
static int find_mac_addr(device_t dev, uint64_t mac);
/* TODO */
static int mac_table_write(device_t dev, uint64_t mac, int idx,
    uint32_t port_map, uint8_t age, int *hash_idx );

static int	get_reg(device_t dev, uint32_t reg, uint32_t *value);
static int 	set_reg(device_t dev, uint32_t reg, uint32_t *value);
static int	set_port_vid(device_t dev, int port, uint16_t pvid);
static int	get_port_vid(device_t dev, int port, uint16_t *pvid);
static int	set_vid(device_t dev, int idx, uint16_t vid);
static int	get_vid(device_t dev, int idx, uint16_t *vid);
static int	set_vlan_ports(device_t dev, int idx, uint32_t memb);
static int	get_vlan_ports(device_t dev, int idx, uint32_t *memb);
static int	set_vlan_untagged_ports(device_t dev, int idx, uint32_t memb);
static int	get_vlan_untagged_ports(device_t dev, int idx, uint32_t *memb);

static int	bcm5325_write(struct b53switch_softc *sc, uint32_t reg,
		    uint64_t val);


static int	bcm5325_vlan_write(struct b53switch_softc *sc, uint16_t vid,
		    uint32_t tports, uint32_t uports);
static int	bcm5325_vlan_read(struct b53switch_softc *sc, uint16_t vid,
		    uint32_t *tports, uint32_t *uports);
static int	bcm5395_vlan_write(struct b53switch_softc *sc, uint16_t vid,
		    uint32_t tports, uint32_t uports);
static int	bcm5395_vlan_read(struct b53switch_softc *sc, uint16_t vid,
		    uint32_t *tports, uint32_t *uports);

#define	PSPHY_WRITE(_sc, _reg, _val)			\
	    MII_SW_WRITE4((_sc), ((PSEUDOPHY_ADDR << 8) | (_reg)), (_val))
#define	PSPHY_READ(_sc, _reg)				\
	    MII_SW_READ4((_sc), ((PSEUDOPHY_ADDR << 8) | (_reg)))

#define	WRITE4(_sc, _reg, _val)			\
	    MII_SW_WRITE4((_sc), ((PSEUDOPHY_ADDR << 8) | (_reg)), (_val))

#define	WRITE(_sc, _reg, _val)	bcm5325_write((_sc), (_reg), (_val))
#define	READ(_sc, _reg, _val)	bcm5325_read((_sc), (_reg), (_val))



static int
bcm5325_write(struct b53switch_softc *sc, uint32_t reg, uint64_t val)
{
	int i;
	uint8_t len, page;

	len = (reg & 0x00ff0000) >> 16;
	page = (reg & 0x0000ff00) >> 8;
	reg &= 0x000000ff;

	PSPHY_WRITE(sc, BCM5325_ACCESS_CONTROL_REG,
		  (((page << ACCESS_CONTROL_PAGE_SHIFT) &
		    ACCESS_CONTROL_PAGE_MASK) | ACCESS_CONTROL_RW));

	for (i = 0; i < len; i+=2){
		PSPHY_WRITE(sc, BCM5325_DATA_15_0_REG + (i/2),
			  (val >> (8*i)) & 0xffff);
	}

	PSPHY_WRITE(sc, BCM5325_RW_CONTROL_REG,
		  ((reg << RW_CONTROL_ADDR_SHIFT) & RW_CONTROL_ADDR_MASK) |
		  RW_CONTROL_WRITE);

	/* is operation finished? */
	for (i = BCM5325_OP_RETRY; i > 0; i --) {
		if ((PSPHY_READ(sc, BCM5325_RW_CONTROL_REG) &
		     RW_CONTROL_OP_MASK) == RW_CONTROL_NOP)
			break;
	}

	/* timed out */
	if (!i) {
		printf("mii_wreg: timeout\n");
		return (EBUSY);
	}

	i = PSPHY_READ(sc, BCM5325_RW_STATUS_REG);
	if (i & 0x0003)
		printf("XXX: reg=%08x BCM5325_RW_STATUS_REG=%d\n", reg, i);

	return (0);
}

static int
bcm5325_read(struct b53switch_softc *sc, uint32_t reg, uint64_t *val)
{

}

static int
bcm5325_switch_probe(device_t dev)
{
	struct child_res_avl *res;

	res = device_get_ivars(dev);

	/* XXX: bcm5325 show at leaset 5 PHYs */
	if (!res->phys)
		return (ENXIO);

	device_set_desc(dev, "BCM5325 family ethernet switch");
	return (BUS_PROBE_DEFAULT);
}

static int
bcm5325_switch_attach(device_t dev)
{
	struct b53switch_softc *sc;
	struct mii_softc	*miisc;
	struct switch_softc	*ssc;
	uint64_t reg;
	uint64_t reg32;

	sc = device_get_softc(dev);
	sc->parent = device_get_parent(dev);
	/* Use BCM5354 mode by default */
	/* XXX: about 5354 we can decide at robo switch core on SSB */
	sc->devid = 0x5325;

	reg = 0;
	READ(sc, SWITCH_DEVICEID, &reg);
	reg32 = reg & 0xfffff;
	if (reg32) {
#if 1
		/* At least BCM53115 return 0x53115 if query in 32bit mode */
		sc->devid = reg32;
#else
		/* XXX: maybe wrong, based on known chips */
		if (reg32 < 0x100)
			sc->devid = reg32 + 0x5300;
		else
			sc->devid = reg32 + 0x50000;
#endif
	}

	resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "devid", &sc->devid);
	device_printf(dev, "\t%switch model is BCM%x\n",
	    ((resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "devid", &sc->devid))?"S":"Hinted s"), sc->devid);

	sc->caps = malloc(sizeof(struct switch_capability), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	if (!sc->caps)
		return (ENXIO);

	switch (sc->devid) {
	case 0x5395:
	case 0x53115:
	case 0x53118:
		sc->vlan_write = &bcm5395_vlan_write;
		sc->vlan_read = &bcm5395_vlan_read;
		sc->vlans = 4096;
		break;
	case 0x5325:
	case 0x5352:
	case 0x5354:
	default:
		sc->vlan_write = &bcm5325_vlan_write;
		sc->vlan_read = &bcm5325_vlan_read;
		sc->vlans = 16;
		break;
	}

	switch (sc->devid) {
	case 0x53118:
	case 0x53115:	/* 53118 w/o ports 6-7 */
	case 0x5325:	/* MII port is port8 */
	case 0x5352:
	case 0x5354:
	case 0x5395:
		sc->caps->ports = sc->ports = 9;
		break;
	case 0x5380:
		/* 8 - 10/100, 2 - 10/100/1000 */
		sc->caps->ports = sc->ports = 10;
		break;
	default:
		/* XXX: trick, last digit of id + 1 MII port */
		sc->caps->ports = sc->ports = (sc->devid % 0xf) + 1;
		break;
	}

	switch (sc->devid) {
	/* XXX Incomplete list of 1000base.* switches */
	case 0x53118:
	case 0x53115:
		break;
	default:
		ssc = device_get_softc(device_get_parent(dev));
		miisc = &ssc->sc_mii;
		/* Remove 1000base.* capabilities for 10/100 switches */
#define	BMSR_EXTCAP	0x0001	/* Extended capability */
		miisc->mii_capabilities &= ~BMSR_EXTCAP;
		miisc->mii_extcapabilities = 0;
		break;
	}
	sc->sc_dev = dev;

#define S_C(x) SWITCH_CAPS_ ## x
	sc->caps->main = S_C(MAIN_PORT_POWER);
	sc->caps->vlan = S_C(VLAN_DOT1Q) |
	    ((sc->vlans << S_C(VLAN_MAX_SHIFT_SHIFT)) &
		S_C(VLAN_MAX_SHIFT_MASK));
	sc->caps->qos = (2 << S_C(QOS_QUEUES_SHIFT)) & S_C(QOS_QUEUES_MASK);
	sc->caps->lacp = 0; /* No LACP caps */
	sc->caps->stp = 0; /* No STP caps */
	sc->caps->acl = 0;
	sc->caps->stat = 0;
#undef S_C



	/* MII port state override (page 0 register 14) */
	READ(sc, PORTMII_STATUS_OVERRIDE , &reg);

	/* Bit 4 enables reverse MII mode */
	if (!(reg & PORTMII_STATUS_REVERSE_MII))
	{
		/* Enable RvMII */
		reg |= PORTMII_STATUS_REVERSE_MII;
		WRITE(sc, PORTMII_STATUS_OVERRIDE, reg);
		/* Read back */
		READ(sc, PORTMII_STATUS_OVERRIDE, &reg);
		if (!(reg & PORTMII_STATUS_REVERSE_MII))
		{
			device_printf(dev, "Unable to set RvMII mode\n");
			bcm5325_switch_detach(dev);
			return (ENXIO);
		}
	}

	/*
	 * XXX: We need prefetch existing sc->base_vlan here.
	 */
	/*
	 * XXX: Avoid default configuration, bootloader must set it or we
	 * must load user defined
	 */

	return (0);
}

static int
bcm5325_switch_detach(device_t dev)
{
	struct b53switch_softc *sc;

	sc = device_get_softc(dev);

	if (sc->caps)
		free(sc->caps, M_DEVBUF);

	return (0);
}

/*
 * Switch capability
 */
static struct switch_capability *
get_caps(device_t dev)
{
	struct b53switch_softc *sc;

	sc = device_get_softc(dev);

	return (sc->caps);
}

/*
 * Variable holding upper 32bits of get_reg/set_reg requests.
 * Accessible with special reg address 0x0fffffff
 */
static uint32_t get_set_upper32 = 0;

static int
get_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct b53switch_softc *sc;
	uint64_t val64;
	int error = 0;

	sc = device_get_softc(dev);

	switch (reg & SWITCH_REG_TYPE_MASK) {
	case SWITCH_REG_TYPE_PHY: /* Same in BCM53xx case */
	case SWITCH_REG_TYPE_RAW:
		*value = MII_SW_READ4(sc, reg);
		return (0);
	}

	if (reg == 0x0ffffffful) {
		*value = get_set_upper32;
	} else if (reg == 0x0ffffffeul) {
		*value = sc->base_vlan;
	} else {
		error = READ(sc, reg, &val64);
		if (((reg & 0x00ff0000) >> 16) > 4)
			printf("\t%08x = %016jx\n", reg, val64);
		if (error == 0) {
			*value = (uint32_t)(val64 & 0xffffffff);
			get_set_upper32 =
			    (uint32_t)((val64 >> 32) & 0xffffffff);
		}
	}
	return (error);
}

static int
set_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct b53switch_softc *sc;
	uint64_t val64;
	uint32_t old;
	int error = 0;

	sc = device_get_softc(dev);

	switch (reg & SWITCH_REG_TYPE_MASK) {
	case SWITCH_REG_TYPE_PHY: /* Same in BCM53xx case */
	case SWITCH_REG_TYPE_RAW:
		old = MII_SW_READ4(sc, reg);
		MII_SW_WRITE4(sc, reg, *value);
		*value = old;
		return (0);
	}

	if (reg == 0x0ffffffful) {
		old = get_set_upper32;
		get_set_upper32 = *value;
		*value = old;
	} else {
		error = READ(sc, reg, &val64);
		if (error == 0) {
			/*
			 * If old value required for 64bits registers,
			 * use get_reg first
			 */
			old = (uint32_t)(val64 & 0xffffffff);
		} else {
			return (error);
		}
		/*
		 * When write 64bits value always set 0x0fffffff reg
		 * to upper 32 bit
		 */
		val64 = ((uint64_t)get_set_upper32 << 32) | (*value);
		error = WRITE(sc, reg, val64);
		if (error == 0)
			return (error);

		*value = old;
	}
	return (error);
}

static int
find_mac_addr(device_t dev, uint64_t mac)
{
	struct b53switch_softc *sc;
	int idx = -1;

	sc = device_get_softc(dev);

	return (idx);
}

static int
mac_table_write(device_t dev, uint64_t mac, int idx, uint32_t port_map,
    uint8_t age, int *hash_idx )
{

	/* TODO */
	return (0);
}

static int
set_port_vid(device_t dev, int port, uint16_t pvid)
{
	struct b53switch_softc *sc;
	int error = 0;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	if (pvid > 0xfff)
		return (EINVAL);

	error = WRITE(sc, VLAN_DEFAULT_PORT_TAG(port), pvid);

	return (error);
}

static int
get_port_vid(device_t dev, int port, uint16_t *pvid)
{
	struct b53switch_softc *sc;
	uint64_t reg;
	int error = 0;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	error = READ(sc, VLAN_DEFAULT_PORT_TAG(port), &reg);
	*pvid = reg & 0xfff;

	return (error);
}

static int
get_port_flags(device_t dev, int port, uint32_t *flags)
{
	struct b53switch_softc *sc;
	uint64_t reg = 0;
	int error = 0;

	*flags = 0;
	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	error = READ(sc, VLAN_DROP_UNTAGGED, &reg);
	if (error)
		return (error);
	if (reg & VLAN_DROP_UNTAGGED_ONPORT(port))
		*flags |= DOT1Q_VLAN_PORT_FLAG_DROP_UNTAGGED;

	return (0);
}

static int
set_port_flags(device_t dev, int port, uint32_t flags)
{
	struct b53switch_softc *sc;
	uint64_t reg;
	int error = 0;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	error = READ(sc, VLAN_DROP_UNTAGGED, &reg);
	if (error)
		return (error);

	if (flags & DOT1Q_VLAN_PORT_FLAG_DROP_UNTAGGED)
		reg |= VLAN_DROP_UNTAGGED_ONPORT(port);
	else
		reg &= ~VLAN_DROP_UNTAGGED_ONPORT(port);

	error = WRITE(sc, VLAN_DROP_UNTAGGED, reg);

	return (error);
}

static int
bcm5325_vlan_write(struct b53switch_softc *sc, uint16_t vid,
		   uint32_t tports, uint32_t uports)
{
	uint64_t reg;
	int error = 0;

	reg = VLAN_RW_VALID |
	    ((tports << VLAN_RW_MEMBER_SHIFT) & VLAN_RW_MEMBER_MASK) |
	    ((uports << VLAN_RW_UNTAGGED_SHIFT) & VLAN_RW_UNTAGGED_MASK);
	error = WRITE(sc, VLAN_WRITE, reg);
	if (error)
		return (error);
	reg = VLAN_TABLE_ACCESS_RW_ENABLE | VLAN_TABLE_ACCESS_WRITE |
	    (vid & VLAN_TABLE_ACCESS_VID_MASK);
	error = WRITE(sc, VLAN_TABLE_ACCESS, reg);

	return (error);
}

static int
bcm5325_vlan_read(struct b53switch_softc *sc, uint16_t vid,
		  uint32_t *tports, uint32_t *uports)
{
	uint64_t reg;
	int error = 0;

	reg = VLAN_TABLE_ACCESS_RW_ENABLE |
	    (vid & VLAN_TABLE_ACCESS_VID_MASK);
	error = WRITE(sc, VLAN_TABLE_ACCESS, reg);
	if (error)
		return (error);

	error = READ(sc, VLAN_READ, &reg);
	if (error)
		return (error);
	if (!(reg & VLAN_RW_VALID))
		return (ENOENT);
	if (tports != NULL)
		*tports = (reg & VLAN_RW_MEMBER_MASK) >> VLAN_RW_MEMBER_SHIFT;
	if (uports != NULL)
		*uports =
		    (reg & VLAN_RW_UNTAGGED_MASK) >> VLAN_RW_UNTAGGED_SHIFT;

	return (error);
}

static int
bcm5395_vlan_write(struct b53switch_softc *sc, uint16_t vid,
		   uint32_t tports, uint32_t uports)
{
	int error = 0;

	error = WRITE(sc, VLAN_TABLE_ENTRY_5395,
	    (uports << VLAN_RW_UNTAG_SHIFT_5395) | tports);
	if (error)
		return (error);
	error = WRITE(sc, VLAN_TABLE_INDX_5395, vid);
	if (error)
		return (error);
	error = WRITE(sc, VLAN_TABLE_ACCESS_5395, VLAN_TABLE_ACCESS_5395_RUN);
	if (error)
		return (error);

	return (error);
}

static int
bcm5395_vlan_read(struct b53switch_softc *sc, uint16_t vid,
		  uint32_t *tports, uint32_t *uports)
{
	uint64_t reg, mask;
	int error = 0;

	error = WRITE(sc, VLAN_TABLE_INDX_5395, vid);
	if (error)
		return (error);
	error = WRITE(sc, VLAN_TABLE_ACCESS_5395,
	    VLAN_TABLE_ACCESS_5395_RUN |VLAN_TABLE_ACCESS_5395_READ);
	if (error)
		return (error);

	error = READ(sc, VLAN_TABLE_ENTRY_5395, &reg);
	if (error)
		return (error);

	mask = (1 << VLAN_RW_UNTAG_SHIFT_5395) - 1;
	if (tports != NULL)
		*tports = reg & mask;
	if (uports != NULL)
		*uports = (reg >> VLAN_RW_UNTAG_SHIFT_5395) & mask;

	return (error);
}

static inline int
bcmXXXX_vlan_write(struct b53switch_softc *sc, uint16_t vid,
		   uint32_t tports, uint32_t uports)
{

	return (sc->vlan_write(sc, vid, tports, uports));
}

static inline int
bcmXXXX_vlan_read(struct b53switch_softc *sc, uint16_t vid,
		  uint32_t *tports, uint32_t *uports)
{

	return (sc->vlan_read(sc, vid, tports, uports));
}
/*
 * set_vid(dev, idx, vid)
 * Define a VLAN, since BCM5325 family use only lower 4-8 bits of VID -
 * idx parameter ignored.
 * VID = base_vlan << 4(or 8, dep on chip) + idx.
 * When base_vlan not equal with previouse value, WARNING displayed.
 */
static int
set_vid(device_t dev, int idx, uint16_t vid)
{
	struct b53switch_softc *sc;
	uint16_t base_vlan_mask;
	int error = 0;

	sc = device_get_softc(dev);
	if (idx > (sc->vlans - 1))
		return (EINVAL);

	base_vlan_mask = ~(sc->vlans - 1);
	if ((vid & base_vlan_mask) != sc->base_vlan) {
		sc->base_vlan = (vid & base_vlan_mask);
		device_printf(sc->sc_dev, "WARNING: Base VLAN changed %04x\n",
			      sc->base_vlan);
	}

	error = bcmXXXX_vlan_read(sc, vid, NULL, NULL);
	if (error == ENOENT) {
		/* Create empty valid VLAN port set */
		bcmXXXX_vlan_write(sc, vid, 0, 0);
		return (0);
	}

	return (error);
}

static int
get_vid(device_t dev, int idx, uint16_t *vid)
{
	struct b53switch_softc *sc = device_get_softc(dev);

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	*vid = sc->base_vlan * sc->vlans + idx;

	return (0);
}

static int
set_vlan_ports(device_t dev, int idx, uint32_t memb)
{
	struct b53switch_softc *sc = device_get_softc(dev);
	uint32_t umemb;
	int error = 0;

	printf("%s: idx=%d, memb=%08x\n", __func__, idx, memb);

	if (idx > (sc->vlans - 1))
		return (EINVAL);
	if (memb & ~((1 << sc->ports) - 1))
		return (EINVAL);

	bcmXXXX_vlan_read(sc, idx /* must be vid */, NULL, &umemb);
	error = bcmXXXX_vlan_write(sc, idx /* must be vid */, memb, umemb);

	return (error);
}

static int
get_vlan_ports(device_t dev, int idx, uint32_t *memb)
{
	struct b53switch_softc *sc = device_get_softc(dev);
	uint32_t reg;
	int error;

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	error = bcmXXXX_vlan_read(sc, idx /* must be vid */, &reg, NULL);
	printf("%s: error=%d idx=%d, memb=%08x\n", __func__, error, idx, reg);
	if (error == ENOENT) {
		*memb = 0;
		return (0);
	}
	if (error)
		return (error);
	*memb = reg;

	return (0);
}

static int
set_vlan_untagged_ports(device_t dev, int idx, uint32_t umemb)
{
	struct b53switch_softc *sc = device_get_softc(dev);
	uint32_t memb;
	int error = 0;

	printf("%s: idx=%d, memb=%08x\n", __func__, idx, umemb);

	if (idx > (sc->vlans - 1))
		return (EINVAL);
	if (memb & ~((1 << sc->ports) - 1))
		return (EINVAL);

	bcmXXXX_vlan_read(sc, idx /* must be vid */, &memb, NULL);
	error = bcmXXXX_vlan_write(sc, idx /* must be vid */, memb | umemb,
	    umemb);

	return (error);
}

static int
get_vlan_untagged_ports(device_t dev, int idx, uint32_t *memb)
{
	struct b53switch_softc *sc = device_get_softc(dev);
	uint32_t reg;
	int error;

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	error = bcmXXXX_vlan_read(sc, idx /* must be vid */, NULL, &reg);
	printf("%s: error=%d idx=%d, memb=%08x\n", __func__, error, idx, reg);
	if (error == ENOENT) {
		*memb = 0;
		return (0);
	}
	if (error)
		return (error);
	*memb = reg;

	return (0);
}

static int
pbvlan_setports(device_t dev, int port,	uint32_t allowed)
{
	struct b53switch_softc *sc;
	int error;

	sc = device_get_softc(dev);

	if (port > (sc->ports - 1))
		return (EINVAL);

	error = WRITE(sc, PBVLAN_ALLOWED_PORTS(port), allowed);
	if (error)
		return (error);

	return (0);
}

static int
pbvlan_getports(device_t dev, int port,	uint32_t *allowed)
{
	struct b53switch_softc *sc;
	uint64_t reg;
	int error;

	sc = device_get_softc(dev);

	if (port > (sc->ports - 1))
		return (EINVAL);

	error = READ(sc, PBVLAN_ALLOWED_PORTS(port), &reg);
	if (error)
		return (error);

	*allowed = (uint32_t)reg;
	return (0);
}

static int
reset_subsys(device_t dev, int subsys)
{
	struct b53switch_softc *sc;
	int error, port;

	sc = device_get_softc(dev);
	error = 0;
	switch (subsys & SWITCH_RESETSUB_MASK) {
	case SWITCH_RESETSUB_SWITCH:
		/* XXX: Hope it will reset any switch */
		error = WRITE(sc, SWITCH_RESET, 0xffff);
		if (error)
			return (error);

		DELAY(10000);

		error = WRITE(sc, SWITCH_RESET, 0x00);
		if (error)
			return (error);
		break;
	case SWITCH_RESETSUB_PORT:
		if ((subsys & SWITCH_RESETSUB_PORT_MASK) ==
		    SWITCH_RESETSUB_ALLPORTS) {
			/* Reset all PHYs */
#ifdef notyet
			for (port = 0; port < sc->ports; port ++)
				reset_port(sc, port);
#endif
		} else {
			/* Reset syngle PHY */
			port = (subsys & SWITCH_RESETSUB_PORT_MASK) >>
			    SWITCH_RESETSUB_PORT_SHIFT;
#ifdef notyet
			reset_port(sc, port);
#endif
		}
		break;
	case SWITCH_RESETSUB_VLANS:
		/* TODO */
		break;
	case SWITCH_RESETSUB_QOS:
		/* TODO */
		break;
	}

	return (error);
}
#endif

static int
b53switch_getvgroup(device_t dev, etherswitch_vlangroup_t *vg)
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
		device_printf(dev, "can't write to VLAN_TABLE_ACCESS reg: %d",
		    error);
		return (error);
	}

	error = b53switch_op(sc, VLAN_READ, &reg, 0);
	if (error) {
		device_printf(dev, "can't read from VLAN_READ reg: %d",
		    error);
		return (error);
	}

	if (!(reg & VLAN_RW_VALID)) {
		device_printf(dev, "not a valid VLAN id: %d",
		    vid);
		return (ENOENT);
	}


	vg->es_vid = vg->es_vlangroup; /* XXX: ??? */
	vg->es_member_ports = B53_UNSHIFT(reg, VLAN_RW_MEMBER);
	vg->es_untagged_ports = B53_UNSHIFT(reg, VLAN_RW_UNTAGGED);
	/* TODO: forwarding */
	vg->es_fid = 0; // RTL8366RB_VMCR_FID(vmcr);

	return (0);
}

static device_method_t b53switch_methods [] =
{
	DEVMETHOD(device_probe,		b53switch_probe),
	DEVMETHOD(device_attach,	b53switch_attach),
	/* bus interface */
	DEVMETHOD(bus_add_child,	device_add_child_ordered),

	/* mii interface */
	DEVMETHOD(miibus_readreg,	b53switch_readreg),
	DEVMETHOD(miibus_writereg,	b53switch_writereg),

//	/* Capability */
//	DEVMETHOD(switch_get_caps,	get_caps),
//	DEVMETHOD(switch_set_reg,	set_reg),
//	DEVMETHOD(switch_get_reg,	get_reg),
//	DEVMETHOD(switch_reset_subsys,	reset_subsys),
//
//	/* MAC address table */
//	DEVMETHOD(switch_find_mac,	find_mac_addr),
//	DEVMETHOD(switch_mac_write,	mac_table_write),
//
//	/* 802.1q */
//	DEVMETHOD(switch_set_pvid,	set_port_vid),
//	DEVMETHOD(switch_get_pvid,	get_port_vid),
//	DEVMETHOD(switch_set_pflags,	set_port_flags),
//	DEVMETHOD(switch_get_pflags,	get_port_flags),
//	DEVMETHOD(switch_set_vid,	set_vid),
//	DEVMETHOD(switch_get_vid,	get_vid),
//	DEVMETHOD(switch_set_vlanports,	set_vlan_ports),
//	DEVMETHOD(switch_get_vlanports,	get_vlan_ports),
//	DEVMETHOD(switch_set_vlanutports,	set_vlan_untagged_ports),
//	DEVMETHOD(switch_get_vlanutports,	get_vlan_untagged_ports),
//
//	/* Port based VLAN */
//	DEVMETHOD(switch_pbvlan_getports,	pbvlan_getports),
//	DEVMETHOD(switch_pbvlan_setports,	pbvlan_setports),

	/* etherswitch interface */
	DEVMETHOD(etherswitch_getinfo,		b53switch_getinfo),
	DEVMETHOD(etherswitch_lock,		b53switch_lock),
	DEVMETHOD(etherswitch_unlock,		b53switch_unlock),
	DEVMETHOD(etherswitch_getport,		b53switch_getport),
#if 0
	DEVMETHOD(etherswitch_setport,		b53switch_setport),

	DEVMETHOD(etherswitch_readreg,		b53switch_readreg_wrapper),
	DEVMETHOD(etherswitch_writereg,		b53switch_writereg_wrapper),
	DEVMETHOD(etherswitch_readphyreg,	b53switch_readphy_wrapper),
	DEVMETHOD(etherswitch_writephyreg,	b53switch_writephy_wrapper),

	DEVMETHOD(etherswitch_setvgroup,	b53switch_setvgroup_wrapper),
#endif
	DEVMETHOD(etherswitch_getvgroup,	b53switch_getvgroup),
	DEVMETHOD_END

};

static devclass_t b53switch_devclass;

DEFINE_CLASS_0(b53switch, b53switch_driver, b53switch_methods,
    sizeof(struct b53switch_softc));

DRIVER_MODULE(b53switch, mdio, b53switch_driver, b53switch_devclass, 0, 0);
DRIVER_MODULE(etherswitch, b53switch, etherswitch_driver, etherswitch_devclass, 0,
    0);
DRIVER_MODULE(miibus, b53switch, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(b53switch, mdio, 1, 1, 1);
