/*
 * bcm_dma_tx.c
 *
 *  Created on: May 24, 2016
 *      Author: mizhka
 */

static int
bcm_dma_tx_start(struct bcm_mac *mac, struct ieee80211_node *ni, struct mbuf *m)
{
#define	BCM_GET_TXHDRCACHE(slot)					\
	&(txhdr_cache[(slot / BCM_TX_SLOTS_PER_FRAME) * BCM_HDRSIZE(mac)])
	struct bcm_dma *dma = &mac->mac_method.dma;
	struct bcm_dma_ring *dr = bcm_dma_select(mac, M_WME_GETAC(m));
	struct bcm_dmadesc_generic *desc;
	struct bcm_dmadesc_meta *mt;
	struct bcm_softc *sc = mac->mac_sc;
	uint8_t *txhdr_cache = (uint8_t *)dr->dr_txhdr_cache;
	int error, slot, backup[2] = { dr->dr_curslot, dr->dr_usedslot };

	BCM_ASSERT_LOCKED(sc);
	KASSERT(!dr->dr_stop, ("%s:%d: fail", __func__, __LINE__));

	/* XXX send after DTIM */

	slot = bcm_dma_getslot(dr);
	dr->getdesc(dr, slot, &desc, &mt);
	KASSERT(mt->mt_txtype == BCM_DMADESC_METATYPE_HEADER,
	    ("%s:%d: fail", __func__, __LINE__));

	error = bcm_set_txhdr(dr->dr_mac, ni, m,
	    (struct bcm_txhdr *)BCM_GET_TXHDRCACHE(slot),
	    BCM_DMA_COOKIE(dr, slot));
	if (error)
		goto fail;
	error = bus_dmamap_load(dr->dr_txring_dtag, mt->mt_dmap,
	    BCM_GET_TXHDRCACHE(slot), BCM_HDRSIZE(mac), bcm_dmamap_callback,
	    &mt->mt_paddr, BUS_DMA_NOWAIT);
	if (error) {
		device_printf(sc->sc_dev, "%s: can't load TX buffer (1) %d\n",
		    __func__, error);
		goto fail;
	}
	bus_dmamap_sync(dr->dr_txring_dtag, mt->mt_dmap,
	    BUS_DMASYNC_PREWRITE);
	dr->setdesc(dr, desc, mt->mt_paddr, BCM_HDRSIZE(mac), 1, 0, 0);
	bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap,
	    BUS_DMASYNC_PREWRITE);

	slot = bcm_dma_getslot(dr);
	dr->getdesc(dr, slot, &desc, &mt);
	KASSERT(mt->mt_txtype == BCM_DMADESC_METATYPE_BODY &&
	    mt->mt_islast == 1, ("%s:%d: fail", __func__, __LINE__));
	mt->mt_m = m;
	mt->mt_ni = ni;

	error = bus_dmamap_load_mbuf(dma->txbuf_dtag, mt->mt_dmap, m,
	    bcm_dmamap_callback_mbuf, &mt->mt_paddr, BUS_DMA_NOWAIT);
	if (error && error != EFBIG) {
		device_printf(sc->sc_dev, "%s: can't load TX buffer (1) %d\n",
		    __func__, error);
		goto fail;
	}
	if (error) {    /* error == EFBIG */
		struct mbuf *m_new;

		m_new = m_defrag(m, M_NOWAIT);
		if (m_new == NULL) {
			device_printf(sc->sc_dev,
			    "%s: can't defrag TX buffer\n",
			    __func__);
			error = ENOBUFS;
			goto fail;
		} else {
			m = m_new;
		}

		mt->mt_m = m;
		error = bus_dmamap_load_mbuf(dma->txbuf_dtag, mt->mt_dmap,
		    m, bcm_dmamap_callback_mbuf, &mt->mt_paddr, BUS_DMA_NOWAIT);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: can't load TX buffer (2) %d\n",
			    __func__, error);
			goto fail;
		}
	}
	bus_dmamap_sync(dma->txbuf_dtag, mt->mt_dmap, BUS_DMASYNC_PREWRITE);
	dr->setdesc(dr, desc, mt->mt_paddr, m->m_pkthdr.len, 0, 1, 1);
	bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap,
	    BUS_DMASYNC_PREWRITE);

	/* XXX send after DTIM */

	dr->start_transfer(dr, bcm_dma_nextslot(dr, slot));
	return (0);
fail:
	dr->dr_curslot = backup[0];
	dr->dr_usedslot = backup[1];
	return (error);
#undef BCM_GET_TXHDRCACHE
}


static struct bcm_dma_ring *
bcm_dma_parse_cookie(struct bcm_mac *mac, const struct bcm_txstatus *status,
    uint16_t cookie, int *slot)
{
	struct bcm_dma *dma = &mac->mac_method.dma;
	struct bcm_dma_ring *dr;
	struct bcm_softc *sc = mac->mac_sc;

	BCM_ASSERT_LOCKED(mac->mac_sc);

	switch (cookie & 0xf000) {
	case 0x1000:
		dr = dma->wme[WME_AC_BK];
		break;
	case 0x2000:
		dr = dma->wme[WME_AC_BE];
		break;
	case 0x3000:
		dr = dma->wme[WME_AC_VI];
		break;
	case 0x4000:
		dr = dma->wme[WME_AC_VO];
		break;
	case 0x5000:
		dr = dma->mcast;
		break;
	default:
		dr = NULL;
		KASSERT(0 == 1,
		    ("invalid cookie value %d", cookie & 0xf000));
	}
	*slot = (cookie & 0x0fff);
	if (*slot < 0 || *slot >= dr->dr_numslots) {
		/*
		 * XXX FIXME: sometimes H/W returns TX DONE events duplicately
		 * that it occurs events which have same H/W sequence numbers.
		 * When it's occurred just prints a WARNING msgs and ignores.
		 */
		KASSERT(status->seq == dma->lastseq,
		    ("%s:%d: fail", __func__, __LINE__));
		device_printf(sc->sc_dev,
		    "out of slot ranges (0 < %d < %d)\n", *slot,
		    dr->dr_numslots);
		return (NULL);
	}
	dma->lastseq = status->seq;
	return (dr);
}


static void
bcm_dma_handle_txeof(struct bwn_mac *mac,
    const struct bwn_txstatus *status)
{
	struct bwn_dma *dma = &mac->mac_method.dma;
	struct bwn_dma_ring *dr;
	struct bwn_dmadesc_generic *desc;
	struct bwn_dmadesc_meta *meta;
	struct bwn_softc *sc = mac->mac_sc;
	int slot;
	int retrycnt = 0;

	BWN_ASSERT_LOCKED(sc);

	dr = bwn_dma_parse_cookie(mac, status, status->cookie, &slot);
	if (dr == NULL) {
		device_printf(sc->sc_dev, "failed to parse cookie\n");
		return;
	}
	KASSERT(dr->dr_tx, ("%s:%d: fail", __func__, __LINE__));

	while (1) {
		KASSERT(slot >= 0 && slot < dr->dr_numslots,
		    ("%s:%d: fail", __func__, __LINE__));
		dr->getdesc(dr, slot, &desc, &meta);

		if (meta->mt_txtype == BWN_DMADESC_METATYPE_HEADER)
			bus_dmamap_unload(dr->dr_txring_dtag, meta->mt_dmap);
		else if (meta->mt_txtype == BWN_DMADESC_METATYPE_BODY)
			bus_dmamap_unload(dma->txbuf_dtag, meta->mt_dmap);

		if (meta->mt_islast) {
			KASSERT(meta->mt_m != NULL,
			    ("%s:%d: fail", __func__, __LINE__));

			/*
			 * If we don't get an ACK, then we should log the
			 * full framecnt.  That may be 0 if it's a PHY
			 * failure, so ensure that gets logged as some
			 * retry attempt.
			 */
			if (status->ack) {
				retrycnt = status->framecnt - 1;
			} else {
				retrycnt = status->framecnt;
				if (retrycnt == 0)
					retrycnt = 1;
			}
			ieee80211_ratectl_tx_complete(meta->mt_ni->ni_vap, meta->mt_ni,
			    status->ack ?
			      IEEE80211_RATECTL_TX_SUCCESS :
			      IEEE80211_RATECTL_TX_FAILURE,
			    &retrycnt, 0);
			ieee80211_tx_complete(meta->mt_ni, meta->mt_m, 0);
			meta->mt_ni = NULL;
			meta->mt_m = NULL;
		} else
			KASSERT(meta->mt_m == NULL,
			    ("%s:%d: fail", __func__, __LINE__));

		dr->dr_usedslot--;
		if (meta->mt_islast)
			break;
		slot = bwn_dma_nextslot(dr, slot);
	}
	sc->sc_watchdog_timer = 0;
	if (dr->dr_stop) {
		KASSERT(bwn_dma_freeslot(dr) >= BWN_TX_SLOTS_PER_FRAME,
		    ("%s:%d: fail", __func__, __LINE__));
		dr->dr_stop = 0;
	}
}


static int
bcm_dma_tx_reset(struct bcm_mac *mac, uint16_t base,
    int type)
{
	struct bcm_softc *sc = mac->mac_sc;
	uint32_t value;
	int i;
	uint16_t offset;

	for (i = 0; i < 10; i++) {
		offset = (type == BCM_DMA_64BIT) ? BCM_DMA64_TXSTATUS :
		    BCM_DMA32_TXSTATUS;
		value = BCM_READ_4(mac, base + offset);
		if (type == BCM_DMA_64BIT) {
			value &= BCM_DMA64_TXSTAT;
			if (value == BCM_DMA64_TXSTAT_DISABLED ||
			    value == BCM_DMA64_TXSTAT_IDLEWAIT ||
			    value == BCM_DMA64_TXSTAT_STOPPED)
				break;
		} else {
			value &= BCM_DMA32_TXSTATE;
			if (value == BCM_DMA32_TXSTAT_DISABLED ||
			    value == BCM_DMA32_TXSTAT_IDLEWAIT ||
			    value == BCM_DMA32_TXSTAT_STOPPED)
				break;
		}
		DELAY(1000);
	}
	offset = (type == BCM_DMA_64BIT) ? BCM_DMA64_TXCTL : BCM_DMA32_TXCTL;
	BCM_WRITE_4(mac, base + offset, 0);
	for (i = 0; i < 10; i++) {
		offset = (type == BCM_DMA_64BIT) ? BCM_DMA64_TXSTATUS :
						   BCM_DMA32_TXSTATUS;
		value = BCM_READ_4(mac, base + offset);
		if (type == BCM_DMA_64BIT) {
			value &= BCM_DMA64_TXSTAT;
			if (value == BCM_DMA64_TXSTAT_DISABLED) {
				i = -1;
				break;
			}
		} else {
			value &= BCM_DMA32_TXSTATE;
			if (value == BCM_DMA32_TXSTAT_DISABLED) {
				i = -1;
				break;
			}
		}
		DELAY(1000);
	}
	if (i != -1) {
		device_printf(sc->sc_dev, "%s: timed out\n", __func__);
		return (ENODEV);
	}
	DELAY(1000);

	return (0);
}
