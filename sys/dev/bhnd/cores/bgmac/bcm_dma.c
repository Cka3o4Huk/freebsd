/*-
 * Copyright (c) 2009-2010 Weongyo Jeong <weongyo@freebsd.org>
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
//
///*
// * The Broadcom Wireless LAN controller driver.
// */
//
//#include "opt_bwn.h"
//#include "opt_wlan.h"
//
//#include <sys/param.h>
//#include <sys/systm.h>
//#include <sys/kernel.h>
//#include <sys/malloc.h>
//#include <sys/module.h>
//#include <sys/endian.h>
//#include <sys/errno.h>
//#include <sys/firmware.h>
//#include <sys/lock.h>
//#include <sys/mutex.h>
//#include <machine/bus.h>
//#include <machine/resource.h>
//#include <sys/bus.h>
//#include <sys/rman.h>
//#include <sys/socket.h>
//#include <sys/sockio.h>
//
//#include <net/ethernet.h>
//#include <net/if.h>
//#include <net/if_var.h>
//#include <net/if_arp.h>
//#include <net/if_dl.h>
//#include <net/if_llc.h>
//#include <net/if_media.h>
//#include <net/if_types.h>
//
//#include <dev/pci/pcivar.h>
//#include <dev/pci/pcireg.h>
//#include <dev/siba/siba_ids.h>
//#include <dev/siba/sibareg.h>
//#include <dev/siba/sibavar.h>
//
//#include <net80211/ieee80211_var.h>
//#include <net80211/ieee80211_radiotap.h>
//#include <net80211/ieee80211_regdomain.h>
//#include <net80211/ieee80211_phy.h>
//#include <net80211/ieee80211_ratectl.h>
//
//#include <dev/bwn/if_bwnreg.h>
//#include <dev/bwn/if_bwnvar.h>
//
//#include <dev/bwn/if_bcm_debug.h>
//#include <dev/bwn/if_bcm_misc.h>
//#include <dev/bwn/if_bcm_util.h>
//#include <dev/bwn/if_bcm_phy_common.h>
//#include <dev/bwn/if_bcm_phy_g.h>
//#include <dev/bwn/if_bcm_phy_lp.h>
//#include <dev/bwn/if_bcm_phy_n.h>
//

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/errno.h>
#include <sys/bus.h>

#include <machine/bus.h>

#include "bcm_dma.h"
#include "bcm_dma_ringvar.h"

void		bcm_dma_init(struct bcm_dma *dma);
void		bcm_dma_stop(struct bcm_dma *dma);
void		bcm_dma_free(struct bcm_dma *dma);

int		bcm_dma_create_tags(device_t dev, struct bcm_dma *dma);
void		bcm_dma_destroy_tags(struct bcm_dma *dma);

int		bcm_dma_mask2type(uint64_t);
uint64_t	bcm_dma_mask(struct bcm_dma *dma);

static int	bcm_dma_gettype(struct bcm_dma *dma);

int
bcm_dma_attach(device_t dev, struct resource *res, struct bcm_dma *dma)
{
	int error;

	dma->dmatype = bcm_dma_gettype(dma);
	error = bcm_dma_create_tags(dev, dma);
	if (error) {
		return (error);
	}
//	dma->wme[WME_AC_BK] = bcm_dma_ringsetup(mac, 0, 1, dma->dmatype);
//	if (!dma->wme[WME_AC_BK])
//		goto fail2;
//
//	dma->wme[WME_AC_BE] = bcm_dma_ringsetup(mac, 1, 1, dma->dmatype);
//	if (!dma->wme[WME_AC_BE])
//		goto fail3;
//
//	dma->wme[WME_AC_VI] = bcm_dma_ringsetup(mac, 2, 1, dma->dmatype);
//	if (!dma->wme[WME_AC_VI])
//		goto fail4;
//
//	dma->wme[WME_AC_VO] = bcm_dma_ringsetup(mac, 3, 1, dma->dmatype);
//	if (!dma->wme[WME_AC_VO])
//		goto fail5;
//
//	dma->mcast = bcm_dma_ringsetup(mac, 4, 1, dma->dmatype);
//	if (!dma->mcast)
//		goto fail6;
	//fail7:	bcm_dma_ringfree(&dma->mcast);
	//fail6:	bcm_dma_ringfree(&dma->wme[WME_AC_VO]);
	//fail5:	bcm_dma_ringfree(&dma->wme[WME_AC_VI]);
	//fail4:	bcm_dma_ringfree(&dma->wme[WME_AC_BE]);
	//fail3:	bcm_dma_ringfree(&dma->wme[WME_AC_BK]);

	dma->rx = bcm_dma_ring_setup(dma, res, 0, 0, dma->dmatype);
	if (!dma->rx) {
		bcm_dma_destroy_tags(dma);
		return (ENXIO);
	}

	bcm_dma_init(dma);

	return (error);
}

int
bcm_dma_create_tags(device_t dev, struct bcm_dma *dma)
{
	int 		error;
	bus_addr_t 	lowaddr;

	printf("bcm_dma_create_tags\n");
	if (dma->dmatype == BCM_DMA_32BIT)
		lowaddr = BUS_SPACE_MAXADDR_32BIT;
	else
		lowaddr = BUS_SPACE_MAXADDR;

	/*
	 * Create top level DMA tag
	 */
	error = bus_dma_tag_create(bus_get_dma_tag(dev),	/* parent */
		    BCM_DMA_ALIGN, 0,		/* alignment, bounds */
		    lowaddr,			/* lowaddr */
		    BUS_SPACE_MAXADDR,		/* highaddr */
		    NULL, NULL,			/* filter, filterarg */
		    BUS_SPACE_MAXSIZE,		/* maxsize */
		    BUS_SPACE_UNRESTRICTED,	/* nsegments */
		    BUS_SPACE_MAXSIZE,		/* maxsegsize */
		    0,				/* flags */
		    NULL, NULL,			/* lockfunc, lockarg */
		    &dma->parent_dtag);
	if (error) {
		device_printf(dev, "can't create parent DMA tag\n");
		return (error);
	}

	/*
	 * Create TX/RX mbuf DMA tag
	 */
	error = bus_dma_tag_create(dma->parent_dtag,
				1,
				0,
				BUS_SPACE_MAXADDR,
				BUS_SPACE_MAXADDR,
				NULL, NULL,
				MCLBYTES,
				1,
				BUS_SPACE_MAXSIZE_32BIT,
				0,
				NULL, NULL,
				&dma->rxbuf_dtag);
	if (error) {
		device_printf(dev, "can't create mbuf DMA tag\n");
		goto fail0;
	}

	error = bus_dma_tag_create(dma->parent_dtag,
				1,
				0,
				BUS_SPACE_MAXADDR,
				BUS_SPACE_MAXADDR,
				NULL, NULL,
				MCLBYTES,
				1,
				BUS_SPACE_MAXSIZE_32BIT,
				0,
				NULL, NULL,
				&dma->txbuf_dtag);
	if (error) {
		device_printf(dev, "can't create mbuf DMA tag\n");
		goto fail1;
	}

	printf("tags[%p]: %p - %p - %p\n", dma, dma->parent_dtag, dma->rxbuf_dtag,
	    dma->txbuf_dtag);

	return (0);

	/*
	 * TODO: add TX path
	 */
//fail2:
//	bus_dma_tag_destroy(dma->txbuf_dtag);
fail1:
fail0:
	printf("error occurred: %d\n", error);
	bcm_dma_destroy_tags(dma);
	return (error);
}

void
bcm_dma_destroy_tags(struct bcm_dma *dma)
{
	printf("bcm_dma_destroy_tags: %p\n", dma);
	if(dma->txbuf_dtag != NULL)
		bus_dma_tag_destroy(dma->txbuf_dtag);
	if(dma->rxbuf_dtag != NULL)
		bus_dma_tag_destroy(dma->rxbuf_dtag);
	if(dma->parent_dtag != NULL)
		bus_dma_tag_destroy(dma->parent_dtag);
}

void
bcm_dma_init(struct bcm_dma *dma)
{
	/* setup TX DMA channels. */
	/* TODO: wme =? QOS */
//	bcm_dma_setup(dma->wme[WME_AC_BK]);
//	bcm_dma_setup(dma->wme[WME_AC_BE]);
//	bcm_dma_setup(dma->wme[WME_AC_VI]);
//	bcm_dma_setup(dma->wme[WME_AC_VO]);
//	bcm_dma_setup(dma->mcast);
	/* setup RX DMA channel. */
	bcm_dma_ringload(dma->rx);
}

void
bcm_dma_stop(struct bcm_dma *dma)
{

//	if ((mac->mac_flags & BCM_MAC_FLAG_DMA) == 0)
//		return;
//	dma = &mac->mac_method.dma;

	bcm_dma_ringstop(dma->rx);
//	bcm_dma_ringstop(&dma->wme[WME_AC_BK]);
//	bcm_dma_ringstop(&dma->wme[WME_AC_BE]);
//	bcm_dma_ringstop(&dma->wme[WME_AC_VI]);
//	bcm_dma_ringstop(&dma->wme[WME_AC_VO]);
//	bcm_dma_ringstop(&dma->mcast);
}

void
bcm_dma_free(struct bcm_dma *dma)
{
//	struct bcm_dma *dma;
//
//	if ((mac->mac_flags & BCM_MAC_FLAG_DMA) == 0)
//		return;
//	dma = &mac->mac_method.dma;

	bcm_dma_ring_free(&dma->rx);
//	bcm_dma_ringfree(&dma->wme[WME_AC_BK]);
//	bcm_dma_ringfree(&dma->wme[WME_AC_BE]);
//	bcm_dma_ringfree(&dma->wme[WME_AC_VI]);
//	bcm_dma_ringfree(&dma->wme[WME_AC_VO]);
//	bcm_dma_ringfree(&dma->mcast);
}

void
bcm_dmamap_callback(void *arg, bus_dma_segment_t *seg, int nseg, int error)
{
	if (!error) {
		KASSERT(nseg == 1, ("too many segments(%d)\n", nseg));
		*((bus_addr_t *)arg) = seg->ds_addr;
	}
}

void
bcm_dmamap_callback_mbuf(void *arg, bus_dma_segment_t *seg, int nseg,
		 bus_size_t mapsz __unused, int error)
{

	bcm_dmamap_callback(arg, seg, nseg, error);
}

static int
bcm_dma_gettype(struct bcm_dma *dma)
{
//	uint32_t tmp;
//	uint16_t base;
//

	// Chip specific logic, for bgmac - 64
//	tmp = bus_read_4(mac, SIBA_TGSHIGH);
//	if (tmp & SIBA_TGSHIGH_DMA64)
//		return (BCM_DMA_64BIT);
//	base = bcm_dma_base(0, 0);
//	BCM_DMA_WRITE(mac, base + BCM_DMA_CTL, BCM_DMA_CTL_ADDREXT_MASK);
//	tmp = BCM_DMA_READ(mac, base + BCM_DMA_CTL);
//	if (tmp & BCM_DMA_CTL_ADDREXT_MASK)
//		return (BCM_DMA_32BIT);

	return (BCM_DMA_64BIT);
}

uint64_t
bcm_dma_mask(struct bcm_dma *dma)
{
	return (BCM_DMA_BIT_MASK(bcm_dma_gettype(dma)));
}

int
bcm_dma_mask2type(uint64_t dmamask)
{
	int	bitpos;

	bitpos = ffs(dmamask + 1);
	if(bitpos == 0)
		bitpos = 64;

	KASSERT(bitpos == 32 || bitpos == 64 || bitpos == 30,
			("%s:%d: fail", __func__, __LINE__));

	return bitpos;
}

uint16_t
bcm_dma_base(int type, int controller_idx, int is_tx)
{
	uint16_t		 base;
	const uint16_t	 	*map;
	int			 size;

	static const uint16_t map64[] = {
		BCM_DMA64_BASE0,
		BCM_DMA64_BASE1,
		BCM_DMA64_BASE2,
		BCM_DMA64_BASE3,
		BCM_DMA64_BASE4,
		BCM_DMA64_BASE5,
	};

	static const uint16_t map32[] = {
		BCM_DMA32_BASE0,
		BCM_DMA32_BASE1,
		BCM_DMA32_BASE2,
		BCM_DMA32_BASE3,
		BCM_DMA32_BASE4,
		BCM_DMA32_BASE5,
	};

	if (type == BCM_DMA_64BIT) {
		size = N(map64);
		map = map64;
		base = (is_tx == 0) ? BCM_DMA64_BASE_RX_SHIFT : 0;
	} else {
		size = N(map32);
		map = map32;
		base = (is_tx == 0) ? BCM_DMA32_BASE_RX_SHIFT : 0;
	}

	/*
	 * Check if controller index belongs to mapXX
	 */
	KASSERT(controller_idx >= 0 && controller_idx < size,
	    ("%s:%d: controller index is out of band", __func__, __LINE__));

	base += map[controller_idx];
	return (base);
}
