/*-
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 * Broadcom Gigabit MAC
 *
 * On Asus RT-N16 GMAC core attaches BCM53115 chip to SoC via GMII. So this driver is
 * MII-based. Information about registers are taken from:
 *      http://bcm-v4.sipsolutions.net/mac-gbit/Registers
 *
 */

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/mutex.h>

#include <sys/kdb.h>

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

#include "miibus_if.h"
#include "mdio_if.h"

#include <dev/mdio/mdio.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miidevs.h"
#include <dev/mii/brgphyreg.h>

#include <machine/bus.h>
#include <machine/resource.h>

#define	BHND_LOGGING	BHND_INFO_LEVEL
#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhnd_ids.h>

#include "bgmac.h"
#include "bgmacvar.h"
#include "bgmacreg.h"

#include "bcm_dma.h"

MALLOC_DEFINE(M_BHND_BGMAC, "bgmac", "Structures allocated by bgmac driver");

struct resource_spec bgmac_rspec[BGMAC_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

struct bhnd_device bgmac_match[] = {
	BHND_DEVICE(GMAC, "BHND Gigabit MAC", NULL),
	BHND_DEVICE_END,
};

/****************************************************************************
 * Prototypes
 ****************************************************************************/
static int	bgmac_probe(device_t dev);
static int	bgmac_attach(device_t dev);

/* MII interface */
static int	bgmac_readreg(device_t dev, int phy, int reg);
static int	bgmac_writereg(device_t dev, int phy, int reg, int val);
static int	bgmac_phyreg_poll(device_t dev, uint32_t reg, uint32_t mask);
static int	bgmac_phyreg_op(device_t dev, phymode op, int phy, int reg,
		    int* val);

/* Interrupt flow */
static void	bgmac_intr(void *arg);

/* NVRAM variables */
static int	bgmac_get_config(struct bgmac_softc *sc);

/* chip manipulations */
static void	bgmac_chip_start_txrx(struct bgmac_softc * sc);
static void	bgmac_chip_stop_txrx(struct bgmac_softc * sc);
static void	bgmac_chip_set_macaddr(struct bgmac_softc * sc);
static void	bgmac_chip_set_cmdcfg(struct bgmac_softc * sc, uint32_t val);
static void	bgmac_chip_set_intr_mask(struct bgmac_softc *sc,
		    enum bgmac_intr_status st);

/* ifnet(9) interface */
static void	bgmac_if_setup(device_t dev);
static void	bgmac_if_init(void* arg);
static void	bgmac_if_init_locked(struct bgmac_softc *sc);
static int	bgmac_if_ioctl(if_t ifp, u_long command, caddr_t data);
static int	bgmac_if_mediachange(struct ifnet *ifp);
static void	bgmac_if_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr);


/****************************************************************************
 * Implementation
 ****************************************************************************/
static int
bgmac_probe(device_t dev)
{
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, bgmac_match, sizeof(bgmac_match[0]));
	if (id == NULL)
		return (ENXIO);

	device_set_desc(dev, id->desc);
	return (BUS_PROBE_DEFAULT);
}

static int
bgmac_attach(device_t dev)
{
	struct bgmac_softc	*sc;
	struct resource		*res[BGMAC_MAX_RSPEC];
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/*
	 * FIXME: move to hints
	 */
	bus_set_resource(dev, SYS_RES_IRQ, 0, 4, 1);

	/* Allocate bus resources */
	error = bus_alloc_resources(dev, bgmac_rspec, res);
	if (error){
		BHND_ERROR_DEV(dev, "can't allocate resources: %d", error);
		return (error);
	}

	sc->mem = res[0];
	sc->irq = res[1];

	BGMACDUMP(sc);

	error = bgmac_get_config(sc);
	if (error) {
		BHND_ERROR_DEV(dev, "can't get bgmac config from NVRAM: %d",
		    error);
		return (ENXIO);
	}

	bgmac_chip_set_macaddr(sc);

	/*
	 * Hook interrupt
	 */
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_NET | INTR_MPSAFE,
			NULL, bgmac_intr, sc, &sc->intrhand);
	if (error) {
		BHND_ERROR_DEV(dev, "can't setup interrupt");
		/*
		 * TODO: cleanup / detach
		 */
		return (error);
	}

	sc->dma = malloc(sizeof(struct bcm_dma), M_BHND_BGMAC, M_WAITOK);
	if (sc->dma == NULL) {
		BHND_ERROR_DEV(dev, "can't allocate memory for bcm_dma");
		/*
		 * TODO: cleanup
		 */
		return (ENOMEM);
	}

	error = bcm_dma_attach(dev, sc->mem, sc->dma);
	if (error) {
		BHND_ERROR_DEV(dev, "error occurried during bcm_dma_attach: %d",
		    error);
		/*
		 * TODO: cleanup
		 */
		return (error);
	}

	bgmac_if_setup(dev);
	BGMAC_LOCK_INIT(sc);

	sc->mdio = device_add_child(dev, "mdio", -1);
	bus_generic_attach(dev);

	return 0;
}

static void
bgmac_chip_set_intr_mask(struct bgmac_softc *sc, enum bgmac_intr_status st)
{
	uint32_t	mask;
	uint32_t	feed;

	if (st & I_OR) {
		mask = bus_read_4(sc->mem, BGMAC_REG_INTERRUPT_MASK);
	} else {
		mask = 0;
	}

	if (st & I_ERR)
		mask |= BGMAC_REG_INTR_STATUS_ERR;
	if (st & I_RX)
		mask |= BGMAC_REG_INTR_STATUS_RX;
	if (st & I_TX)
		mask |= BGMAC_REG_INTR_STATUS_TX;

	bus_write_4(sc->mem, BGMAC_REG_INTERRUPT_MASK, mask);
	feed = bus_read_4(sc->mem, BGMAC_REG_INTERRUPT_MASK);
	BHND_TRACE_DEV(sc->dev, "bgmac_intr: 0x%x", feed);
}

static void
bgmac_chip_set_cmdcfg(struct bgmac_softc *sc, uint32_t val)
{
	uint32_t	tmp;

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp | BGMAC_REG_CMD_CFG_RESET);
	/* hope it's enough for reset */
	DELAY(100);

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp | val);

	DELAY(100);

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp & ~BGMAC_REG_CMD_CFG_RESET);

	DELAY(100);
}

static void
bgmac_chip_set_macaddr(struct bgmac_softc * sc)
{

	/*
	 * Write MAC address
	 */
	bus_write_4(sc->mem, 0x80c, *((uint32_t*)sc->addr));
	bus_write_4(sc->mem, 0x810, *(((uint16_t*)sc->addr)+2));
}

static void
bgmac_chip_start_txrx(struct bgmac_softc * sc)
{

	BGMAC_ASSERT_LOCKED(sc);
	BHND_INFO_DEV(sc->dev, "starting TX / RX");
	bgmac_chip_set_cmdcfg(sc, BGMAC_REG_CMD_CFG_RX | BGMAC_REG_CMD_CFG_TX);
	bgmac_chip_set_intr_mask(sc, I_ERR | I_RX | I_TX);
	return;
}

static void
bgmac_chip_stop_txrx(struct bgmac_softc * sc)
{

	BGMAC_ASSERT_LOCKED(sc);
	BHND_INFO_DEV(sc->dev, "stop TX / RX");
	bgmac_chip_set_cmdcfg(sc, 0);
	bgmac_chip_set_intr_mask(sc, I_ERR);
	return;
}

/*
 * Setup interface
 */
static void
bgmac_if_setup(device_t dev)
{
	struct bgmac_softc	*sc;
	struct ifnet		*ifp;

	sc = device_get_softc(dev);
	ifp = sc->ifp = if_alloc(IFT_ETHER);
	ifp->if_softc = sc;

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setinitfn(ifp, bgmac_if_init);
	if_setioctlfn(ifp, bgmac_if_ioctl);
	if_setstartfn(ifp, bgmac_if_start);

	ifmedia_init(&sc->ifmedia, 0, bgmac_if_mediachange, bgmac_if_mediastatus);
	ifmedia_add(&sc->ifmedia, IFM_ETHER | IFM_100_T2 | IFM_FDX, 0, NULL);
	ifmedia_set(&sc->ifmedia, IFM_ETHER | IFM_100_T2 | IFM_FDX);

	ether_ifattach(ifp, sc->addr);
	ifp->if_capabilities = ifp->if_capenable = IFCAP_RXCSUM | IFCAP_TXCSUM;

	//IFCAP_VLAN_MTU | IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_HWCSUM;
	//IFCAP_VLAN_HWFILTER | ;

	return;
}

static int
bgmac_if_mediachange(struct ifnet *ifp)
{
	struct bgmac_softc	*sc;
	struct ifmedia		*ifm;
	struct ifmedia_entry	*ife;

	sc = ifp->if_softc;
	ifm = &sc->ifmedia;
	ife = ifm->ifm_cur;

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return (EINVAL);

	if (IFM_SUBTYPE(ife->ifm_media) == IFM_AUTO) {
		device_printf(sc->dev, "AUTO is not supported for multiphy MAC");
		return (EINVAL);
	}

	/*
	 * Ignore everything
	 */
	return (0);
}

static void
bgmac_if_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr)
{

	ifmr->ifm_status = IFM_AVALID | IFM_ACTIVE;
	/* TODO: retrieve mode & duplex info from softc */
	ifmr->ifm_active = IFM_ETHER | IFM_100_T2 | IFM_FDX;
}

static void
bgmac_if_init(void* arg)
{
	struct bgmac_softc	*sc;

	sc = (struct bgmac_softc*) arg;

	BGMAC_LOCK(sc);
	bgmac_if_init_locked(sc);
	BGMAC_UNLOCK(sc);
}

static void
bgmac_if_init_locked(struct bgmac_softc *sc)
{

	/* TODO: promiscios mode => ioctl */
	BHND_DEBUG_DEV(sc->dev, "init driver");
	/* set number of interrupts per frame */
	bus_write_4(sc->mem, BGMAC_REG_INTR_RECV_LAZY,
	    1 << BGMAC_REG_INTR_RECV_LAZY_FC_SHIFT);

	bgmac_chip_set_intr_mask(sc, I_ERR | I_OR);

	sc->ifp->if_drv_flags |= IFF_DRV_RUNNING;
	sc->ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
}

static int
bgmac_if_ioctl(if_t ifp, u_long command, caddr_t data)
{
	struct bgmac_softc	*sc;
	struct ifreq		*ifr;
	int			 error;

	sc = if_getsoftc(ifp);
	ifr = (struct ifreq *) data;
	error = 0;

	switch (command) {
	case SIOCSIFFLAGS:
		BGMAC_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				bgmac_chip_start_txrx(sc);
			else
			{
				/* TODO: add detach */
				bgmac_if_init_locked(sc);
				bgmac_chip_start_txrx(sc);
			}
		} else if (ifp->if_drv_flags & IFF_DRV_RUNNING)
			bgmac_chip_stop_txrx(sc);
		BGMAC_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		/* XXX: implement SIOCDELMULTI */
		error = 0;
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->ifmedia, command);
		break;
	default:
		error = ether_ioctl(ifp, command, data);
		break;
	}

	return (error);
}

/**************************** MII BUS Functions *****************************/

int bgmac_phyreg_poll(device_t dev,uint32_t reg, uint32_t mask){
	struct bgmac_softc	*sc;
	int			 i;
	uint32_t		 val;

	sc = device_get_softc(dev);
	i = BGMAC_TIMEOUT;

	/* Poll for the register to complete. */
	for (; i > 0; i--) {
		DELAY(10);
		val = bus_read_4(sc->mem, reg);
		if ((val & mask) != 0)
			continue;
		/* Success */
		DELAY(5);
		return (0);
	}
	return (-1);
}

static int
bgmac_phyreg_op(device_t dev, phymode op, int phy, int reg, int* val)
{
	struct bgmac_softc	*sc;
	uint32_t		 tmp;

	sc = device_get_softc(dev);

	/* Set address on PHY control register */
	tmp = bus_read_4(sc->mem, BGMAC_REG_PHY_CONTROL);
	tmp = (tmp & (~BGMAC_REG_PHY_CONTROL_ADDR)) | phy;

	bus_write_4(sc->mem, BGMAC_REG_PHY_CONTROL, tmp);

	/* Send header (first 16 bytes) over MII */
	tmp = BGMAC_REG_PHY_ACCESS_START;
	if (op == PHY_WRITE) {
		tmp |= BGMAC_REG_PHY_ACCESS_WRITE;
		tmp |= ((*val) & BGMAC_REG_PHY_ACCESS_DATA);
	}

	tmp |= (phy << BGMAC_REG_PHY_ACCESS_ADDR_SHIFT);
	tmp |= (reg << BGMAC_REG_PHY_ACCESS_REG_SHIFT);

	bus_write_4(sc->mem, BGMAC_REG_PHY_ACCESS, tmp);

	/* Wait while operation is finished */
	if (bgmac_phyreg_poll(dev, BGMAC_REG_PHY_ACCESS,
	    BGMAC_REG_PHY_ACCESS_START)) {
		return (-1);
	}

	if (op == PHY_READ) {
		/* Read rest of 16 bytes back */
		tmp = bus_read_4(sc->mem, BGMAC_REG_PHY_ACCESS);
		tmp &= BGMAC_REG_PHY_ACCESS_DATA;
		*val = tmp;
	}

	return (0);
}

static int
bgmac_readreg(device_t dev, int phy, int reg)
{
	int	tmp;

	if (bgmac_phyreg_op(dev, PHY_READ, phy, reg, &tmp)) {
		BHND_ERROR_DEV(dev, "phy_readreg: phy=%x reg=%x", phy, reg);
		return (-1);
	}

	return (tmp);
}

static int
bgmac_writereg(device_t dev, int phy, int reg, int val)
{
	int	tmp;

	tmp = val;
	if (bgmac_phyreg_op(dev, PHY_WRITE, phy, reg, &tmp)) {
		BHND_ERROR_DEV(dev, "phy_writereg: phy=%x reg=%x", phy, reg);
		return (-1);
	}

	return (0);
}

/* Interrupt handler */
static void
bgmac_intr(void *arg)
{
	struct bgmac_softc	*sc;
	uint32_t		 intr_status;

	sc = (struct bgmac_softc*)arg;

	BGMAC_LOCK(sc);

	intr_status = bus_read_4(sc->mem, BGMAC_REG_INTR_STATUS);
	bus_write_4(sc->mem, BGMAC_REG_INTR_STATUS, intr_status);

	BHND_TRACE_DEV(sc->dev, "bgmac_intr_status: 0x%x", intr_status);

	/* disable interrupt for a while */
	bgmac_chip_set_intr_mask(sc, 0);

	if (intr_status & BGMAC_REG_INTR_STATUS_TX) {
		BHND_TRACE_DEV(sc->dev, "TX!");
		bcm_dma_tx(sc->dma->wme[0]);
		intr_status &= ~BGMAC_REG_INTR_STATUS_TX;
	}

	if (intr_status & BGMAC_REG_INTR_STATUS_RX) {
		BHND_TRACE_DEV(sc->dev, "RX!");
		bcm_dma_rx(sc->dma->rx);
		intr_status &= ~BGMAC_REG_INTR_STATUS_RX;
	}

	if (intr_status & BGMAC_REG_INTR_STATUS_ERR_OVER) {
		BHND_ERROR_DEV(sc->dev, "Overflow!");
		BGMACDUMP(sc);
		bcm_dma_rx(sc->dma->rx);
		intr_status &= ~BGMAC_REG_INTR_STATUS_ERR_OVER;
	}

	if (intr_status & BGMAC_REG_INTR_STATUS_ERR_DESC){
		BHND_ERROR_DEV(sc->dev, "ERROR!");
		BGMACDUMP(sc);
		kdb_enter("bgmac_error", "unknown error: descriptor?");
		intr_status &= ~BGMAC_REG_INTR_STATUS_ERR_OVER;
	}

	/* enable interrupt */
	bgmac_chip_set_intr_mask(sc, I_ERR | I_RX | I_TX);
	BGMAC_UNLOCK(sc);
	return;
}

/* Get configuration from NVRAM */
static int
bgmac_get_config(struct bgmac_softc *sc)
{
	char	*tmp;
	char	 macaddr[18];

	tmp = kern_getenv("sb/1/macaddr");
	if (tmp == NULL)
		return (-1);

	if (strlen(tmp) != strlen("00:00:00:00:00:00"))
		return (-2);

	strcpy(macaddr, tmp);

	for (int i = 2; i < strlen(macaddr); i+=3) {
		if (macaddr[i] != ':')
			return (-3);
		macaddr[i] = '\0';
	}

	tmp = macaddr;
	for (int i = 0; i < 6; i++) {
		sc->addr[i] = (u_char)strtoul(tmp, NULL, 16);
		tmp += 3;
	}

	return (0);
}

void
bgmac_rxeof(struct device *dev, struct mbuf *m, struct bcm_rx_header *rxhdr)
{
	struct bgmac_softc	*sc;
	struct ifnet		*ifp;

	/* TODO: early draft - need more attention */
	sc = device_get_softc(dev);
	ifp = sc->ifp;

	BGMAC_ASSERT_LOCKED(sc);

	m->m_pkthdr.rcvif = ifp;
	BGMAC_UNLOCK(sc);
	(*ifp->if_input)(ifp, m);
	BGMAC_LOCK(sc);
}

/**
 * Driver metadata
 */
static device_method_t bgmac_methods[] = {
		DEVMETHOD(device_probe,	 	bgmac_probe),
		DEVMETHOD(device_attach, 	bgmac_attach),
		/** miibus interface **/
		DEVMETHOD(miibus_readreg, 	bgmac_readreg),
		DEVMETHOD(miibus_writereg, 	bgmac_writereg),

		/** MDIO interface **/
		DEVMETHOD(mdio_readreg,		bgmac_readreg),
		DEVMETHOD(mdio_writereg,	bgmac_writereg),

		DEVMETHOD_END
};

devclass_t bgmac_devclass;

DEFINE_CLASS_0(bgmac, bgmac_driver, bgmac_methods, sizeof(struct bgmac_softc));

DRIVER_MODULE(bgmac, bhnd, bgmac_driver, bgmac_devclass, 0, 0);
DRIVER_MODULE(mdio, bgmac, mdio_driver, mdio_devclass, 0, 0);

MODULE_VERSION(bgmac, 1);
