/*
 * bcm_dmaring.c
 *
 *  Created on: May 24, 2016
 *      Author: mizhka
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/errno.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <machine/bus.h>

#define	BHND_LOGGING	BHND_INFO_LEVEL
#include <dev/bhnd/bhnd_debug.h>

#include "bcm_dma_reg.h"
#include "bcm_dma_desc.h"
#include "bcm_dma_ops.h"
#include "bcm_dma_ringvar.h"

/*
 * Internal
 */

static void	bcm_dma_ring_free_memory(struct bcm_dma_ring *dr);
static void	bcm_dma_ring_free_descbufs(struct bcm_dma_ring *dr);
static void	bcm_dma_ring_free_descbuf(struct bcm_dmadesc_meta *meta);

struct bcm_dma_ring *
bcm_dma_ring_setup(struct bcm_dma *dma, struct resource *res, int ctl_id,
		int is_tx, int type)
{
	struct bcm_dma_ring		*dr;
	struct bcm_dmadesc_generic	*desc;
	struct bcm_dmadesc_meta		*mt;
	int				 error, i;

	BHND_DEBUG_DEV(rman_get_device(res), "setup of ring: %s[%d]",
	    (is_tx) ? "TX" : "RX", ctl_id);

	/*
	 * Allocate memory for new ring
	 */
	dr = malloc(sizeof(*dr), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (dr == NULL)
		return (dr);

	/*
	 * Allocate meta's for descriptors / slots
	 */
	dr->dr_numslots = (is_tx > 0) ? BCM_TXRING_SLOTS : BCM_RXRING_SLOTS;
	dr->dr_meta = malloc(dr->dr_numslots * sizeof(struct bcm_dmadesc_meta),
	    M_DEVBUF, M_NOWAIT | M_ZERO);
	if (dr->dr_meta == NULL)
		goto fail0;

	dr->dr_type = type;
	dr->res = res;
	dr->dma = dma;
	KASSERT(dr->res != NULL, ("panic"));
	dr->dr_base = bcm_dma_base(type, ctl_id, is_tx);
	dr->dr_index = ctl_id;

	dr->suspend = bcm_dma_suspend;
	dr->resume = bcm_dma_resume;

	if (type == BCM_DMA_64BIT) {
		dr->getdesc = bcm_dma_64_getdesc;
		dr->setdesc = bcm_dma_64_setdesc;
		dr->start_transfer = bcm_dma_64_start_transfer;
		dr->get_curslot = bcm_dma_64_get_curslot;
		dr->set_curslot = bcm_dma_64_set_curslot;
	} else {
		dr->getdesc = bcm_dma_32_getdesc;
		dr->setdesc = bcm_dma_32_setdesc;
		dr->start_transfer = bcm_dma_32_start_transfer;
		dr->get_curslot = bcm_dma_32_get_curslot;
		dr->set_curslot = bcm_dma_32_set_curslot;
	}

	if (is_tx) {
		dr->dr_is_tx = 1;
		dr->dr_curslot = 0;
		dr->dr_usedslot = dr->dr_numslots;
	} else {
		KASSERT(dr->dr_index == 0, ("%s:%d: fail", __func__, __LINE__));

		/* TODO: correct fake dr_rx_bufsize */
		dr->dr_rx_bufsize = 2342;
		dr->dr_frameoffset = BCM_FRAME_OFFSET;
//		switch (mac->mac_fw.fw_hdr_format) {
//		case BCM_FW_HDR_351:
//		case BCM_FW_HDR_410:
//			dr->dr_rx_bufsize =
//			    BCM_DMA0_RX_BUFFERSIZE_FW351;
//			dr->dr_frameoffset =
//			    BCM_DMA0_RX_FRAMEOFFSET_FW351;
//			break;
//		case BCM_FW_HDR_598:
//			dr->dr_rx_bufsize =
//			    BCM_DMA0_RX_BUFFERSIZE_FW598;
//			dr->dr_frameoffset =
//			    BCM_DMA0_RX_FRAMEOFFSET_FW598;
//			break;
//		}
	}

	error = bcm_dma_ring_alloc(dma, dr);
	if (error)
		goto fail1;

	if (is_tx) {
		/*
		 * TODO: uncomment TX path
		 */
//		/*
//		 * Assumption: BCM_TXRING_SLOTS can be divided by
//		 * BCM_TX_SLOTS_PER_FRAME
//		 */
//		KASSERT(BCM_TXRING_SLOTS % BCM_TX_SLOTS_PER_FRAME == 0,
//		    ("%s:%d: fail", __func__, __LINE__));
//
//		dr->dr_txhdr_cache = contigmalloc(
//		    (dr->dr_numslots / BCM_TX_SLOTS_PER_FRAME) *
//		    BCM_MAXTXHDRSIZE, M_DEVBUF, M_ZERO,
//		    0, BUS_SPACE_MAXADDR, 8, 0);
//		if (dr->dr_txhdr_cache == NULL) {
//			device_printf(sc->sc_dev,
//			    "can't allocate TX header DMA memory\n");
//			goto fail1;
//		}
//
		/*
		 * Create TX ring DMA stuffs
		 */
//		error = bus_dma_tag_create(dma->parent_dtag,
//				    BCM_ALIGN, 0,
//				    BUS_SPACE_MAXADDR,
//				    BUS_SPACE_MAXADDR,
//				    NULL, NULL,
//				    BCM_HDRSIZE(mac),
//				    1,
//				    BUS_SPACE_MAXSIZE_32BIT,
//				    0,
//				    NULL, NULL,
//				    &dr->dr_txring_dtag);
//		if (error) {
//			device_printf(sc->sc_dev,
//			    "can't create TX ring DMA tag: TODO frees\n");
//			goto fail2;
//		}

		for (i = 0; i < dr->dr_numslots; i++) {
//			dr->getdesc(dr, i, &desc, &mt);
//
//			mt->mt_txtype = BCM_DMADESC_METATYPE_HEADER;
//			mt->mt_m = NULL;
//			mt->mt_ni = NULL;
//			mt->mt_islast = 0;
//			error = bus_dmamap_create(dr->dr_txring_dtag, 0,
//			    &mt->mt_dmap);
//			if (error) {
//				device_printf(sc->sc_dev,
//				     "can't create RX buf DMA map\n");
//				goto fail2;
//			}
//
			dr->getdesc(dr, i, &desc, &mt);

//			mt->mt_txtype = BCM_DMADESC_METATYPE_BODY;
			mt->mt_m = NULL;
//			mt->mt_ni = NULL;
			mt->mt_islast = 1;
			error = bus_dmamap_create(dma->txbuf_dtag, 0,
			    &mt->mt_dmap);
			if (error) {
				device_printf(rman_get_device(dr->res),
				     "can't create TX buf DMA map\n");
				goto out; /* XXX wrong! */
			}
		}
	} else {
		error = bus_dmamap_create(dma->rxbuf_dtag, 0,
		    &dr->dr_spare_dmap);
		if (error) {
			device_printf(rman_get_device(dr->res),
			    "can't create RX buf DMA map\n");
			goto out;		/* XXX wrong! */
		}

		printf("dr->dr_numslots: %d\n",dr->dr_numslots);
		for (i = 0; i < dr->dr_numslots; i++) {
			dr->getdesc(dr, i, &desc, &mt);
			error = bus_dmamap_create(dma->rxbuf_dtag, 0,
			    &mt->mt_dmap);
			if (error) {
				device_printf(rman_get_device(dr->res),
				    "can't create RX buf DMA map\n");
				goto out;	/* XXX wrong! */
			}
			error = bcm_dma_rx_newbuf(dr, desc, mt, 1);
			if (error) {
				device_printf(rman_get_device(dr->res),
				    "failed to allocate RX buf\n");
				goto out;	/* XXX wrong! */
			}
		}

		bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap,
		    BUS_DMASYNC_PREWRITE);
	}

      out:
	return (dr);

//fail2:
//	if (dr->dr_txhdr_cache != NULL) {
//		contigfree(dr->dr_txhdr_cache,
//		    (dr->dr_numslots / BCM_TX_SLOTS_PER_FRAME) *
//		    BCM_MAXTXHDRSIZE, M_DEVBUF);
//	}
fail1:
	free(dr->dr_meta, M_DEVBUF);
fail0:
	free(dr, M_DEVBUF);
	return (NULL);
}

int
bcm_dma_ring_alloc(struct bcm_dma *dma, struct bcm_dma_ring *dr)
{
	bus_dma_tag_t		*tagp;
	bus_dmamap_t		*mapp;
	void			**vaddrp;
	bus_addr_t		*paddrp;
	int			 error;

	BHND_DEBUG_DEV(rman_get_device(dr->res), "allocate & load DMA descriptor"
	    " ring %s[%d]", (dr->dr_is_tx) ? "TX" : "RX", dr->dr_index);

	tagp = &dr->dr_ring_dtag;
	mapp = &dr->dr_ring_dmap;
	vaddrp = &dr->dr_ring_descbase;
	paddrp = &dr->dr_ring_dmabase;

	/*
	 * Create DMA tag for description ring
	 */
	error = bus_dma_tag_create(dma->parent_dtag,
			    BCM_DMA_ALIGN, 0,
			    BUS_SPACE_MAXADDR,
			    BUS_SPACE_MAXADDR,
			    NULL, NULL,
			    BCM_DMA_RINGMEMSIZE,
			    1,
			    BUS_SPACE_MAXSIZE_32BIT,
			    0,
			    NULL, NULL,
			    tagp);
	if (error) {
		device_printf(rman_get_device(dr->res),
		    "can't create TX ring DMA tag: TODO frees\n");
		return (-1);
	}

	error = bus_dmamem_alloc(*tagp, vaddrp, BUS_DMA_WAITOK | BUS_DMA_ZERO,
			mapp);
	if (error) {
		device_printf(rman_get_device(dr->res),
		    "can't allocate DMA mem: TODO frees\n");
		return (-1);
	}
	error = bus_dmamap_load(*tagp, *mapp, *vaddrp, BCM_DMA_RINGMEMSIZE,
	    bcm_dmamap_callback, paddrp, BUS_DMA_NOWAIT);

	if (error) {
		device_printf(rman_get_device(dr->res),
		    "can't load DMA mem: TODO free\n");
		return (-1);
	}

	return (0);
}

void
bcm_dma_ring_free(struct bcm_dma_ring **dr)
{

	if (dr == NULL)
		return;

	bcm_dma_ring_free_descbufs(*dr);
	bcm_dma_ring_free_memory(*dr);
/*
 * TODO:
 */
//	if ((*dr)->dr_txhdr_cache != NULL) {
//		contigfree((*dr)->dr_txhdr_cache,
//		    ((*dr)->dr_numslots / BCM_TX_SLOTS_PER_FRAME) *
//		    BCM_MAXTXHDRSIZE, M_DEVBUF);
//	}
	free((*dr)->dr_meta, M_DEVBUF);
	free(*dr, M_DEVBUF);

	*dr = NULL;
}

static void
bcm_dma_ring_free_memory(struct bcm_dma_ring *dr)
{

	bus_dmamap_unload(dr->dr_ring_dtag, dr->dr_ring_dmap);
	bus_dmamem_free(dr->dr_ring_dtag, dr->dr_ring_descbase,
	    dr->dr_ring_dmap);
}

static void
bcm_dma_ring_free_descbufs(struct bcm_dma_ring *dr)
{
	struct bcm_dmadesc_generic 	*desc;
	struct bcm_dmadesc_meta 	*meta;
	device_t			 dev;
//	struct bcm_mac *mac = dr->dr_mac;
//	struct bcm_dma *dma = &mac->mac_method.dma;
//	struct bcm_softc *sc = mac->mac_sc;
	int i;

	dev = rman_get_device(dr->res);

	if (!dr->dr_usedslot)
		return;
	for (i = 0; i < dr->dr_numslots; i++) {
		dr->getdesc(dr, i, &desc, &meta);

		if (meta->mt_m == NULL) {
			if (!dr->dr_is_tx)
				device_printf(dev, "%s: not TX?\n", __func__);
			continue;
		}
		if (dr->dr_is_tx) {
//			if (meta->mt_txtype == BCM_DMADESC_METATYPE_HEADER)
//				bus_dmamap_unload(dr->dr_txring_dtag,
//				    meta->mt_dmap);
//			else if (meta->mt_txtype == BCM_DMADESC_METATYPE_BODY)
			bus_dmamap_unload(dr->dma->txbuf_dtag, meta->mt_dmap);
		} else
			bus_dmamap_unload(dr->dma->rxbuf_dtag, meta->mt_dmap);
		bcm_dma_ring_free_descbuf(meta);
	}
}

static void
bcm_dma_ring_free_descbuf(struct bcm_dmadesc_meta *meta)
{

	if (meta->mt_m != NULL) {
		m_freem(meta->mt_m);
		meta->mt_m = NULL;
	}
	/*
	 * TODO: 802.11 code
	 */
//	if (meta->mt_ni != NULL) {
//		ieee80211_free_node(meta->mt_ni);
//		meta->mt_ni = NULL;
//	}
}

void
bcm_dma_ring_load(struct bcm_dma_ring *dr)
{
	uint64_t	ring64;
	uint32_t 	addrext, ring32, value;
	uint32_t 	trans;
	device_t	dev;

	dev = rman_get_device(dr->res);

	BHND_DEBUG_DEV(dev, "ring_load[%p]", dr);

	trans = 0;

	if (dr->dr_is_tx)
		dr->dr_curslot = 0;
	else
		dr->dr_usedslot = dr->dr_numslots;

	if (dr->dr_type == BCM_DMA_64BIT) {
		ring64 = (uint64_t)(dr->dr_ring_dmabase);
		BHND_DEBUG_DEV(dev, "dr_ring_dmabase: 0x%jx\n", ring64);

		addrext = ((ring64 >> 32) & BCM_DMA_ADDR_MASK) >> 30;

		BCM_DMA_WRITE(dr, BCM_DMA_CTL, 0);
		DELAY(30);

		value = (dr->dr_is_tx) ? 0 :
			    BCM_DMA_SHIFT(dr->dr_frameoffset, BCM_DMA_CTL_RXFROFF);
		value |= BCM_DMA_CTL_ENABLE;
		value |= 0x800; /* disable parity - TODO: move to define*/
		value |= BCM_DMA_SHIFT(addrext, BCM_DMA_CTL_ADDREXT);
		BHND_DEBUG_DEV(dev, "CTL: 0x%x->0x%x", dr->dr_base + BCM_DMA_CTL,
		    value);
		BCM_DMA_WRITE(dr, BCM_DMA_CTL, value);

		BCM_DMA_WRITE(dr, BCM_DMA64_RINGLO, (ring64 & 0xffffffff));
		BCM_DMA_WRITE(dr, BCM_DMA64_RINGHI, ((ring64 >> 32) &
		    ~BCM_DMA_ADDR_MASK) | (trans << 1));

		if (!dr->dr_is_tx)
			BCM_DMA_WRITE(dr, BCM_DMA64_INDEX, dr->dr_numslots *
			    sizeof(struct bcm_dmadesc64));

	} else {
		ring32 = (uint32_t)(dr->dr_ring_dmabase);
		addrext = (ring32 & BCM_DMA_ADDR_MASK) >> 30;

		BCM_DMA_WRITE(dr, BCM_DMA_CTL, 0);
		DELAY(30);

		value = (dr->dr_is_tx) ? 0 :
			    (dr->dr_frameoffset << BCM_DMA_CTL_RXFROFF_SHIFT);
		value |= BCM_DMA_CTL_ENABLE;
		value |= BCM_DMA_SHIFT(addrext, BCM_DMA_CTL_ADDREXT);
		BCM_DMA_WRITE(dr, BCM_DMA_CTL, value);

		BCM_DMA_WRITE(dr, BCM_DMA32_RING,
		    (ring32 & ~BCM_DMA_ADDR_MASK) | trans);
		if (!dr->dr_is_tx)
			BCM_DMA_WRITE(dr, BCM_DMA32_INDEX, dr->dr_numslots *
			    sizeof(struct bcm_dmadesc32));
	}
}

void
bcm_dma_ring_stop(struct bcm_dma_ring *dr)
{

	if (dr == NULL)
		return;

	if (dr->dr_is_tx) {
		/*
		 * TODO: add TX path
		 */
		//bcm_dma_tx_reset(dr->dr_mac, dr->dr_base, dr->dr_type);
		if (dr->dr_type == BCM_DMA_64BIT) {
			BCM_DMA_WRITE(dr, BCM_DMA64_RINGLO, 0);
			BCM_DMA_WRITE(dr, BCM_DMA64_RINGHI, 0);
		} else
			BCM_DMA_WRITE(dr, BCM_DMA32_RING, 0);
	} else {
		bcm_dma_rx_reset(dr, dr->dr_base, dr->dr_type);
		if (dr->dr_type == BCM_DMA_64BIT) {
			BCM_DMA_WRITE(dr, BCM_DMA64_RINGLO, 0);
			BCM_DMA_WRITE(dr, BCM_DMA64_RINGHI, 0);
		} else
			BCM_DMA_WRITE(dr, BCM_DMA32_RING, 0);
	}
}

int
bcm_dmaring_get_freeslot(struct bcm_dma_ring *dr)
{
	BCM_ASSERT_LOCKED(dr->res);

	return (dr->dr_numslots - dr->dr_usedslot);
}

int
bcm_dmaring_get_curslot(struct bcm_dma_ring *dr)
{
//	BCM_ASSERT_LOCKED(dr->res);

//	KASSERT(slot >= -1 && slot <= dr->dr_numslots - 1,
//	    ("%s:%d: fail", __func__, __LINE__));
	return (dr->dr_curslot);
}

int
bcm_dmaring_get_nextslot(struct bcm_dma_ring *dr, int slot)
{
	BCM_ASSERT_LOCKED(dr->res);

	KASSERT(slot >= -1 && slot <= dr->dr_numslots - 1,
	    ("%s:%d: fail", __func__, __LINE__));
	if (slot == dr->dr_numslots - 1)
		return (0);
	return (slot + 1);
}
