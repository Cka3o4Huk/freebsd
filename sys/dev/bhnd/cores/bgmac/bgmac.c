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

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miidevs.h"
#include <dev/mii/brgphyreg.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhnd_ids.h>

#include "bgmacvar.h"
#include "bgmacreg.h"

struct resource_spec bgmac_rspec[BGMAC_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

struct bhnd_device bgmac_match[] = {
	BHND_DEVICE(GMAC, "BHND Gigabit MAC", NULL, NULL),
	BHND_DEVICE_END,
};


/****************************************************************************
 * Prototypes
 ****************************************************************************/

static int	bgmac_probe(device_t dev);
static int	bgmac_attach(device_t dev);

/*
 * MII interface
 */
static int	bgmac_readreg(device_t dev, int phy, int reg);
static int	bgmac_writereg(device_t dev, int phy, int reg, int val);

static int	bgmac_phyreg_poll(device_t dev, uint32_t reg, uint32_t mask);
static int	bgmac_phyreg_op(device_t dev, phymode op, int phy, int reg,
		    int* val);

/*
 * Driver callbacks for media status and change requests.
 */
static int	bgmac_change(struct ifnet *);
static void	bgmac_stat(struct ifnet *, struct ifmediareq *req);

/**
 * Interrupt flow
 */
static void	bgmac_intr(void *arg);
static int	bgmac_get_config(struct bgmac_softc *sc);

/**
 * Internals
 */
static int	bgmac_setup_interface(device_t dev);

/*
 * **************************** Implementation ****************************
 */


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

	error = bgmac_setup_interface(dev);
	/* TODO
	 *  - init interrupt handler
	 *  -
	 */

	bus_write_4(sc->mem, BGMAC_REG_INTR_RECV_LAZY,
			1 << BGMAC_REG_INTR_RECV_LAZY_FC_SHIFT);
	bus_write_4(sc->mem, BGMAC_REG_INTR_STATUS, BGMAC_REG_INTR_STATUS_ALL);
	uint32_t tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG,
	    tmp | BGMAC_REG_CMD_CFG_TX |BGMAC_REG_CMD_CFG_RX );

	return 0;
}

/*
 * Setup interface
 */
static int
bgmac_setup_interface(device_t dev)
{
	struct bgmac_softc	*sc;
	int			 error;
	struct ifnet		*ifp;

	sc = device_get_softc(dev);
	ifp = sc->ifp	= if_alloc(IFT_ETHER);
	ifp->if_softc = sc;

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));

	error = bgmac_get_config(sc);
	if (error) {
		BHND_ERROR_DEV(dev, "can't get bgmac config from NVRAM: %d",
		    error);
		return (ENXIO);
	}

	error = mii_attach(dev, &sc->miibus, ifp, bgmac_change, bgmac_stat,
	    BMSR_DEFCAPMASK, MII_PHY_ANY, MII_OFFSET_ANY, 0);
	if (error) {
		BHND_ERROR_DEV(dev, "mii_attach failed: %d", error);
		return error;
	}

	ether_ifattach(ifp, sc->addr);

	ifp->if_capabilities = ifp->if_capenable = 0;

	/*
	 * Hook interrupt
	 */
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_NET | INTR_MPSAFE,
			NULL, bgmac_intr, sc, &sc->intrhand);

	/*
	 * TODO: ifmedia_init / add / set
	 */

	bgmac_dma_alloc(sc);

	return (0);
}

static int
bgmac_dma_alloc(struct bgmac_softc *sc)
{
	int	err;

	/*
	 * parent tag.  Apparently the chip cannot handle any DMA address
	 * greater than 1GB.
	 */
	err = bus_dma_tag_create(bus_get_dma_tag(sc->dev), /* parent */
	    1, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR, 		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    BUS_SPACE_MAXSIZE_32BIT,	/* maxsize */
	    0,				/* nsegments */
	    BUS_SPACE_MAXSIZE_32BIT,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->parent_tag);

	if (err) {
		BHND_ERROR_DEV(sc->dev, "can't create parent DMA tag: %d", err);
		return (err);
	}

	err = bus_dma_tag_create(sc->parent_tag, /* parent */
	    0x1000, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR, 		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    0x1000,			/* maxsize */                                                                                                                                                                                                                                                     rjxnz,	/* maxsize */
	    1,				/* nsegments */
	    BUS_SPACE_MAXSIZE_32BIT,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->ring_tag);

	err = bus_dmamem_alloc(sc->ring_tag,
				&sc->buf, BUS_DMA_NOWAIT,
				   &sc->ring_map);
	KASSERT(err==0, "panic");

	err = bus_dmamap_load(sc->ring_tag, sc->ring_map, &sc->rxdesc_ring,
			0x1000, bgmac_dma_ring_addr, &sc->rxdesc_ring_busaddr,
			BUS_DMA_NOWAIT);

	KASSERT(err==0, "panic");

	return (0);
}

static void
bgmac_dma_ring_addr(void *arg, bus_dma_segment_t *seg, int nseg, int error)
{
	if (!error) {
		KASSERT(nseg == 1, ("too many segments(%d)\n", nseg));
		*((bus_addr_t *)arg) = seg->ds_addr;
	}
}


/*
 * **************************** MII BUS Functions ****************************
 */

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
	tmp = (tmp & (~BGMAC_REG_PHY_ACCESS_ADDR)) | phy;

	bus_write_4(sc->mem, BGMAC_REG_PHY_CONTROL, tmp);

	/* Send header (first 16 bytes) over MII */
	tmp = BGMAC_REG_PHY_ACCESS_START;
	if (op == PHY_WRITE) {
		tmp |= BGMAC_REG_PHY_ACCESS_WRITE;
		tmp |= (*val & BGMAC_REG_PHY_ACCESS_DATA);
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
	int tmp;

	if (bgmac_phyreg_op(dev, PHY_READ, phy, reg, &tmp)) {
		return (-1);
	}

	return (tmp);
}

static int
bgmac_writereg(device_t dev, int phy, int reg, int val)
{
	int tmp;

	tmp = val;
	if (bgmac_phyreg_op(dev, PHY_WRITE, phy, reg, &tmp)) {
		return (-1);
	}

	return (0);
}

/**
 * Media
 */

static int
bgmac_change(struct ifnet *ifp){
	device_printf(((struct bgmac_softc*)ifp->if_softc)->dev, "change!\n");
	return (0);
}

static void
bgmac_stat(struct ifnet *ifp, struct ifmediareq *req)
{
	device_printf(((struct bgmac_softc*)ifp->if_softc)->dev, "stat!\n");
	return;
}

/**
 * Intr
 */

static void
bgmac_intr(void *arg)
{
	struct bgmac_softc	*sc;

	sc = (struct bgmac_softc*)arg;
	/**
	 * Read status
	 * Handle rx
	 * Handle tr
	 * Handle errors
	 */
	device_printf(sc->dev, "bgmac_intr\n");
	return;
}

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

/**
 * Driver metadata
 */

static device_method_t bgmac_methods[] = {
		DEVMETHOD(device_probe,	 	bgmac_probe),
		DEVMETHOD(device_attach, 	bgmac_attach),
		/** miibus interface **/
		DEVMETHOD(miibus_readreg, 	bgmac_readreg),
		DEVMETHOD(miibus_writereg, 	bgmac_writereg),

		DEVMETHOD_END
};

devclass_t bhnd_bgmac_devclass;

DEFINE_CLASS_0(bhnd_bgmac, bgmac_driver, bgmac_methods,
		sizeof(struct bgmac_softc));
DRIVER_MODULE(bhnd_bgmac, bhnd, bgmac_driver, bhnd_bgmac_devclass, 0, 0);
DRIVER_MODULE(miibus, bhnd_bgmac, miibus_driver, miibus_devclass, 0, 0);
MODULE_VERSION(bhnd_bgmac, 1);
