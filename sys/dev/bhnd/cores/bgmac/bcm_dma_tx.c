/*
 * bcm_dma_tx.c
 *
 *  Created on: May 24, 2016
 *      Author: mizhka
 */

#define	BHND_LOGGING	BHND_TRACE_LEVEL

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/errno.h>
#include <sys/kdb.h>

#include <machine/bus.h>
#include <machine/resource.h>

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

#include <dev/bhnd/bhnd_debug.h>

#include "bgmac.h"
#include "bgmacvar.h"
#include "bgmacreg.h"

#include "bcm_dma.h"
#include "bcm_dma_ringvar.h"

int
bcm_dma_tx_start(struct bcm_dma *dma, struct mbuf *m)
{
	device_t			 dev;
	struct bcm_dma_ring		*dr;
	struct bcm_dmadesc_generic	*desc;
	struct bcm_dmadesc_meta		*mt;

	bus_dma_segment_t		 segs;
	int				 nsegs, slot;
	int				 error;

//	uint8_t *txhdr_cache = (uint8_t *)dr->dr_txhdr_cache;
//	, slot, backup[2] = { dr->dr_curslot, dr->dr_usedslot };

//	BCM_ASSERT_LOCKED(sc);
//	KASSERT(!dr->dr_stop, ("%s:%d: fail", __func__, __LINE__));

	/* XXX send after DTIM */

	dr = dma->wme[0];

	dev = rman_get_device(dr->res);
	slot = bcm_dmaring_get_curslot(dr);
	dr->getdesc(dr, slot, &desc, &mt);
//	KASSERT(mt->mt_txtype == BCM_DMADESC_METATYPE_HEADER,
//	    ("%s:%d: fail", __func__, __LINE__));

//	error = bcm_set_txhdr(dr->dr_mac, ni, m,
//	    (struct bcm_txhdr *)BCM_GET_TXHDRCACHE(slot),
//	    BCM_DMA_COOKIE(dr, slot));
//	if (error)
//		goto fail;

	BHND_DEBUG_DEV(dev, "TX slot[%d]: tag %p, map %p", slot,
	    dr->dr_ring_dtag, mt->mt_dmap);

	BHND_DEBUG_DEV(dev, "TX mbuf[%d]", m->m_len);

	error = bus_dmamap_load_mbuf_sg(dma->txbuf_dtag, mt->mt_dmap, m, &segs,
			&nsegs, BUS_DMA_NOWAIT);
	if (error) {
		BHND_ERROR_DEV(dev, "can't load TX buffer: %d", error);
		goto fail;
	}

	BHND_TRACE_DEV(dev, "TX buffer loaded as: 0x%x (0x%x)", segs.ds_addr,
	    segs.ds_len);

	mt->mt_paddr = segs.ds_addr;

//	error = bus_dmamap_load(dr->dr_txring_dtag, mt->mt_dmap,
//	    BCM_GET_TXHDRCACHE(slot), BCM_HDRSIZE(mac), bcm_dmamap_callback,
//	    &mt->mt_paddr, BUS_DMA_NOWAIT);
//	if (error) {
//		device_printf(sc->sc_dev, "%s: can't load TX buffer (1) %d\n",
//		    __func__, error);
//		goto fail;
//	}
	bus_dmamap_sync(dma->txbuf_dtag, mt->mt_dmap, BUS_DMASYNC_PREWRITE);
	dr->setdesc(dr, desc, mt->mt_paddr, m->m_pkthdr.len, 1, 1, 1);
	bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap, BUS_DMASYNC_PREWRITE);
	dr->dr_curslot++;
//	slot = bcm_dma_getslot(dr);
//	dr->getdesc(dr, slot, &desc, &mt);
//	KASSERT(mt->mt_txtype == BCM_DMADESC_METATYPE_BODY &&
//	    mt->mt_islast == 1, ("%s:%d: fail", __func__, __LINE__));
//	mt->mt_m = m;
//	mt->mt_ni = ni;
//
//	error = bus_dmamap_load_mbuf(dma->txbuf_dtag, mt->mt_dmap, m,
//	    bcm_dmamap_callback_mbuf, &mt->mt_paddr, BUS_DMA_NOWAIT);
//	if (error && error != EFBIG) {
//		device_printf(sc->sc_dev, "%s: can't load TX buffer (1) %d\n",
//		    __func__, error);
//		goto fail;
//	}
//	if (error) {    /* error == EFBIG */
//		struct mbuf *m_new;
//
//		m_new = m_defrag(m, M_NOWAIT);
//		if (m_new == NULL) {
//			device_printf(sc->sc_dev,
//			    "%s: can't defrag TX buffer\n",
//			    __func__);
//			error = ENOBUFS;
//			goto fail;
//		} else {
//			m = m_new;
//		}
//
//		mt->mt_m = m;
//		error = bus_dmamap_load_mbuf(dma->txbuf_dtag, mt->mt_dmap,
//		    m, bcm_dmamap_callback_mbuf, &mt->mt_paddr, BUS_DMA_NOWAIT);
//		if (error) {
//			device_printf(sc->sc_dev,
//			    "%s: can't load TX buffer (2) %d\n",
//			    __func__, error);
//			goto fail;
//		}
//	}
//	bus_dmamap_sync(dma->txbuf_dtag, mt->mt_dmap, BUS_DMASYNC_PREWRITE);
//	dr->setdesc(dr, desc, mt->mt_paddr, m->m_pkthdr.len, 0, 1, 1);
//	bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap,
//	    BUS_DMASYNC_PREWRITE);

	/* XXX send after DTIM */

	dr->start_transfer(dr, bcm_dmaring_get_nextslot(dr, slot));
	return (0);
fail:
/* TODO: support fail */
//	dr->dr_curslot = backup[0];
//	dr->dr_usedslot = backup[1];
	return (error);
}

//static struct bcm_dma_ring *
//bcm_dma_parse_cookie(struct bcm_mac *mac, const struct bcm_txstatus *status,
//    uint16_t cookie, int *slot)
//{
//	struct bcm_dma *dma = &mac->mac_method.dma;
//	struct bcm_dma_ring *dr;
//	struct bcm_softc *sc = mac->mac_sc;
//
//	BCM_ASSERT_LOCKED(mac->mac_sc);
//
//	switch (cookie & 0xf000) {
//	case 0x1000:
//		dr = dma->wme[WME_AC_BK];
//		break;
//	case 0x2000:
//		dr = dma->wme[WME_AC_BE];
//		break;
//	case 0x3000:
//		dr = dma->wme[WME_AC_VI];
//		break;
//	case 0x4000:
//		dr = dma->wme[WME_AC_VO];
//		break;
//	case 0x5000:
//		dr = dma->mcast;
//		break;
//	default:
//		dr = NULL;
//		KASSERT(0 == 1,
//		    ("invalid cookie value %d", cookie & 0xf000));
//	}
//	*slot = (cookie & 0x0fff);
//	if (*slot < 0 || *slot >= dr->dr_numslots) {
//		/*
//		 * XXX FIXME: sometimes H/W returns TX DONE events duplicately
//		 * that it occurs events which have same H/W sequence numbers.
//		 * When it's occurred just prints a WARNING msgs and ignores.
//		 */
//		KASSERT(status->seq == dma->lastseq,
//		    ("%s:%d: fail", __func__, __LINE__));
//		device_printf(sc->sc_dev,
//		    "out of slot ranges (0 < %d < %d)\n", *slot,
//		    dr->dr_numslots);
//		return (NULL);
//	}
//	dma->lastseq = status->seq;
//	return (dr);
//}


void
bcm_dma_tx(struct bcm_dma_ring *dr)
{
	struct bcm_dma			*dma;
	struct bcm_dmadesc_generic	*desc;
	struct bcm_dmadesc_meta		*meta;
	int				 slot;

	/* TODO: locking */
	dma = dr->dma;
	KASSERT(dr->dr_is_tx > 0, ("%s:%d: fail", __func__, __LINE__));

	/* Get slot free for driver, i.e. device sent all data */
	slot = bcm_dmaring_get_freeslot(dr);

	KASSERT(slot >= 0 && slot < dr->dr_numslots,
	    ("%s:%d: fail - %d", __func__, __LINE__, slot));
	dr->getdesc(dr, slot, &desc, &meta);

	bus_dmamap_unload(dma->txbuf_dtag, meta->mt_dmap);
	meta->mt_m = NULL;

	dr->dr_usedslot--;
	if (dr->dr_usedslot == 0)
		dr->dr_usedslot = dr->dr_numslots;
}


//static int
//bcm_dma_tx_reset(struct bcm_mac *mac, uint16_t base,
//    int type)
//{
//	struct bcm_softc *sc = mac->mac_sc;
//	uint32_t value;
//	int i;
//	uint16_t offset;
//
//	for (i = 0; i < 10; i++) {
//		offset = (type == BCM_DMA_64BIT) ? BCM_DMA64_TXSTATUS :
//		    BCM_DMA32_TXSTATUS;
//		value = BCM_READ_4(mac, base + offset);
//		if (type == BCM_DMA_64BIT) {
//			value &= BCM_DMA64_TXSTAT;
//			if (value == BCM_DMA64_TXSTAT_DISABLED ||
//			    value == BCM_DMA64_TXSTAT_IDLEWAIT ||
//			    value == BCM_DMA64_TXSTAT_STOPPED)
//				break;
//		} else {
//			value &= BCM_DMA32_TXSTATE;
//			if (value == BCM_DMA32_TXSTAT_DISABLED ||
//			    value == BCM_DMA32_TXSTAT_IDLEWAIT ||
//			    value == BCM_DMA32_TXSTAT_STOPPED)
//				break;
//		}
//		DELAY(1000);
//	}
//	offset = (type == BCM_DMA_64BIT) ? BCM_DMA64_TXCTL : BCM_DMA32_TXCTL;
//	BCM_WRITE_4(mac, base + offset, 0);
//	for (i = 0; i < 10; i++) {
//		offset = (type == BCM_DMA_64BIT) ? BCM_DMA64_TXSTATUS :
//						   BCM_DMA32_TXSTATUS;
//		value = BCM_READ_4(mac, base + offset);
//		if (type == BCM_DMA_64BIT) {
//			value &= BCM_DMA64_TXSTAT;
//			if (value == BCM_DMA64_TXSTAT_DISABLED) {
//				i = -1;
//				break;
//			}
//		} else {
//			value &= BCM_DMA32_TXSTATE;
//			if (value == BCM_DMA32_TXSTAT_DISABLED) {
//				i = -1;
//				break;
//			}
//		}
//		DELAY(1000);
//	}
//	if (i != -1) {
//		device_printf(sc->sc_dev, "%s: timed out\n", __func__);
//		return (ENODEV);
//	}
//	DELAY(1000);
//
//	return (0);
//}
