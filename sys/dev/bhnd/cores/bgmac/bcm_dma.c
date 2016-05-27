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

#include "bcm_dma.h"

static void	bcm_dma_init(struct bcm_mac *);
static void	bcm_dma_rxdirectfifo(struct bcm_mac *, int, uint8_t);
static int	bcm_dma_mask2type(uint64_t);
static uint64_t	bcm_dma_mask(struct bcm_mac *);
static uint16_t	bcm_dma_base(int, int);
static void	bcm_dma_ringfree(struct bcm_dma_ring **);
static void	bcm_dma_32_getdesc(struct bcm_dma_ring *,
		    int, struct bcm_dmadesc_generic **,
		    struct bcm_dmadesc_meta **);
static void	bcm_dma_32_setdesc(struct bcm_dma_ring *,
		    struct bcm_dmadesc_generic *, bus_addr_t, uint16_t, int,
		    int, int);
static void	bcm_dma_32_start_transfer(struct bcm_dma_ring *, int);
static void	bcm_dma_32_suspend(struct bcm_dma_ring *);
static void	bcm_dma_32_resume(struct bcm_dma_ring *);
static int	bcm_dma_32_get_curslot(struct bcm_dma_ring *);
static void	bcm_dma_32_set_curslot(struct bcm_dma_ring *, int);
static void	bcm_dma_64_getdesc(struct bcm_dma_ring *,
		    int, struct bcm_dmadesc_generic **,
		    struct bcm_dmadesc_meta **);
static void	bcm_dma_64_setdesc(struct bcm_dma_ring *,
		    struct bcm_dmadesc_generic *, bus_addr_t, uint16_t, int,
		    int, int);
static void	bcm_dma_64_start_transfer(struct bcm_dma_ring *, int);
static void	bcm_dma_64_suspend(struct bcm_dma_ring *);
static void	bcm_dma_64_resume(struct bcm_dma_ring *);
static int	bcm_dma_64_get_curslot(struct bcm_dma_ring *);
static void	bcm_dma_64_set_curslot(struct bcm_dma_ring *, int);
static int	bcm_dma_allocringmemory(struct bcm_dma_ring *);
static void	bcm_dma_setup(struct bcm_dma_ring *);
static void	bcm_dma_free_ringmemory(struct bcm_dma_ring *);
static void	bcm_dma_cleanup(struct bcm_dma_ring *);
static void	bcm_dma_free_descbufs(struct bcm_dma_ring *);
static int	bcm_dma_tx_reset(struct bcm_mac *, uint16_t, int);
static void	bcm_dma_rx(struct bcm_dma_ring *);
static int	bcm_dma_rx_reset(struct bcm_mac *, uint16_t, int);
static void	bcm_dma_free_descbuf(struct bcm_dma_ring *,
		    struct bcm_dmadesc_meta *);
static void	bcm_dma_set_redzone(struct bcm_dma_ring *, struct mbuf *);
static int	bcm_dma_gettype(struct bcm_mac *);
static void	bcm_dma_ring_addr(void *, bus_dma_segment_t *, int, int);
static int	bcm_dma_freeslot(struct bcm_dma_ring *);
static int	bcm_dma_nextslot(struct bcm_dma_ring *, int);
static void	bcm_dma_rxeof(struct bcm_dma_ring *, int *);
static int	bcm_dma_newbuf(struct bcm_dma_ring *,
		    struct bcm_dmadesc_generic *, struct bcm_dmadesc_meta *,
		    int);
static void	bcm_dma_buf_addr(void *, bus_dma_segment_t *, int,
		    bus_size_t, int);
static uint8_t	bcm_dma_check_redzone(struct bcm_dma_ring *, struct mbuf *);
static void	bcm_dma_handle_txeof(struct bcm_mac *,
		    const struct bcm_txstatus *);
static int	bcm_dma_tx_start(struct bcm_mac *, struct ieee80211_node *,
		    struct mbuf *);
static int	bcm_dma_getslot(struct bcm_dma_ring *);
static struct bcm_dma_ring *bcm_dma_select(struct bcm_mac *,
		    uint8_t);
static int	bcm_dma_attach(struct bcm_mac *);
static struct bcm_dma_ring *bcm_dma_ringsetup(struct bcm_mac *,
		    int, int, int);
static struct bcm_dma_ring *bcm_dma_parse_cookie(struct bcm_mac *,
		    const struct bcm_txstatus *, uint16_t, int *);
static void	bcm_dma_free(struct bcm_mac *);

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
	    BCM_GET_TXHDRCACHE(slot), BCM_HDRSIZE(mac), bcm_dma_ring_addr,
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
	    bcm_dma_buf_addr, &mt->mt_paddr, BUS_DMA_NOWAIT);
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
		    m, bcm_dma_buf_addr, &mt->mt_paddr, BUS_DMA_NOWAIT);
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

static void
bcm_dma_rxdirectfifo(struct bcm_mac *mac, int idx, uint8_t enable)
{
	uint32_t ctl;
	int type;
	uint16_t base;

	type = bcm_dma_mask2type(bcm_dma_mask(mac));
	base = bcm_dma_base(type, idx);
	if (type == BCM_DMA_64BIT) {
		ctl = BCM_READ_4(mac, base + BCM_DMA64_RXCTL);
		ctl &= ~BCM_DMA64_RXDIRECTFIFO;
		if (enable)
			ctl |= BCM_DMA64_RXDIRECTFIFO;
		BCM_WRITE_4(mac, base + BCM_DMA64_RXCTL, ctl);
	} else {
		ctl = BCM_READ_4(mac, base + BCM_DMA32_RXCTL);
		ctl &= ~BCM_DMA32_RXDIRECTFIFO;
		if (enable)
			ctl |= BCM_DMA32_RXDIRECTFIFO;
		BCM_WRITE_4(mac, base + BCM_DMA32_RXCTL, ctl);
	}
}

static uint64_t
bcm_dma_mask(struct bcm_mac *mac)
{
	uint32_t tmp;
	uint16_t base;

	tmp = BCM_READ_4(mac, SIBA_TGSHIGH);
	if (tmp & SIBA_TGSHIGH_DMA64)
		return (BCM_DMA_BIT_MASK(64));
	base = bcm_dma_base(0, 0);
	BCM_WRITE_4(mac, base + BCM_DMA32_TXCTL, BCM_DMA32_TXADDREXT_MASK);
	tmp = BCM_READ_4(mac, base + BCM_DMA32_TXCTL);
	if (tmp & BCM_DMA32_TXADDREXT_MASK)
		return (BCM_DMA_BIT_MASK(32));

	return (BCM_DMA_BIT_MASK(30));
}

static int
bcm_dma_mask2type(uint64_t dmamask)
{

	if (dmamask == BCM_DMA_BIT_MASK(30))
		return (BCM_DMA_30BIT);
	if (dmamask == BCM_DMA_BIT_MASK(32))
		return (BCM_DMA_32BIT);
	if (dmamask == BCM_DMA_BIT_MASK(64))
		return (BCM_DMA_64BIT);
	KASSERT(0 == 1, ("%s:%d: fail", __func__, __LINE__));
	return (BCM_DMA_30BIT);
}

static uint16_t
bcm_dma_base(int type, int controller_idx)
{
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
		KASSERT(controller_idx >= 0 && controller_idx < N(map64),
		    ("%s:%d: fail", __func__, __LINE__));
		return (map64[controller_idx]);
	}
	KASSERT(controller_idx >= 0 && controller_idx < N(map32),
	    ("%s:%d: fail", __func__, __LINE__));
	return (map32[controller_idx]);
}

static void
bcm_dma_init(struct bcm_mac *mac)
{
	struct bcm_dma *dma = &mac->mac_method.dma;

	/* setup TX DMA channels. */
	bcm_dma_setup(dma->wme[WME_AC_BK]);
	bcm_dma_setup(dma->wme[WME_AC_BE]);
	bcm_dma_setup(dma->wme[WME_AC_VI]);
	bcm_dma_setup(dma->wme[WME_AC_VO]);
	bcm_dma_setup(dma->mcast);
	/* setup RX DMA channel. */
	bcm_dma_setup(dma->rx);
}

static struct bcm_dma_ring *
bcm_dma_ringsetup(struct bcm_mac *mac, int controller_index,
    int for_tx, int type)
{
	struct bcm_dma *dma = &mac->mac_method.dma;
	struct bcm_dma_ring *dr;
	struct bcm_dmadesc_generic *desc;
	struct bcm_dmadesc_meta *mt;
	struct bcm_softc *sc = mac->mac_sc;
	int error, i;

	dr = malloc(sizeof(*dr), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (dr == NULL)
		goto out;
	dr->dr_numslots = BCM_RXRING_SLOTS;
	if (for_tx)
		dr->dr_numslots = BCM_TXRING_SLOTS;

	dr->dr_meta = malloc(dr->dr_numslots * sizeof(struct bcm_dmadesc_meta),
	    M_DEVBUF, M_NOWAIT | M_ZERO);
	if (dr->dr_meta == NULL)
		goto fail0;

	dr->dr_type = type;
	dr->dr_mac = mac;
	dr->dr_base = bcm_dma_base(type, controller_index);
	dr->dr_index = controller_index;
	if (type == BCM_DMA_64BIT) {
		dr->getdesc = bcm_dma_64_getdesc;
		dr->setdesc = bcm_dma_64_setdesc;
		dr->start_transfer = bcm_dma_64_start_transfer;
		dr->suspend = bcm_dma_64_suspend;
		dr->resume = bcm_dma_64_resume;
		dr->get_curslot = bcm_dma_64_get_curslot;
		dr->set_curslot = bcm_dma_64_set_curslot;
	} else {
		dr->getdesc = bcm_dma_32_getdesc;
		dr->setdesc = bcm_dma_32_setdesc;
		dr->start_transfer = bcm_dma_32_start_transfer;
		dr->suspend = bcm_dma_32_suspend;
		dr->resume = bcm_dma_32_resume;
		dr->get_curslot = bcm_dma_32_get_curslot;
		dr->set_curslot = bcm_dma_32_set_curslot;
	}
	if (for_tx) {
		dr->dr_tx = 1;
		dr->dr_curslot = -1;
	} else {
		if (dr->dr_index == 0) {
			switch (mac->mac_fw.fw_hdr_format) {
			case BCM_FW_HDR_351:
			case BCM_FW_HDR_410:
				dr->dr_rx_bufsize =
				    BCM_DMA0_RX_BUFFERSIZE_FW351;
				dr->dr_frameoffset =
				    BCM_DMA0_RX_FRAMEOFFSET_FW351;
				break;
			case BCM_FW_HDR_598:
				dr->dr_rx_bufsize =
				    BCM_DMA0_RX_BUFFERSIZE_FW598;
				dr->dr_frameoffset =
				    BCM_DMA0_RX_FRAMEOFFSET_FW598;
				break;
			}
		} else
			KASSERT(0 == 1, ("%s:%d: fail", __func__, __LINE__));
	}

	error = bcm_dma_allocringmemory(dr);
	if (error)
		goto fail2;

	if (for_tx) {
		/*
		 * Assumption: BCM_TXRING_SLOTS can be divided by
		 * BCM_TX_SLOTS_PER_FRAME
		 */
		KASSERT(BCM_TXRING_SLOTS % BCM_TX_SLOTS_PER_FRAME == 0,
		    ("%s:%d: fail", __func__, __LINE__));

		dr->dr_txhdr_cache = contigmalloc(
		    (dr->dr_numslots / BCM_TX_SLOTS_PER_FRAME) *
		    BCM_MAXTXHDRSIZE, M_DEVBUF, M_ZERO,
		    0, BUS_SPACE_MAXADDR, 8, 0);
		if (dr->dr_txhdr_cache == NULL) {
			device_printf(sc->sc_dev,
			    "can't allocate TX header DMA memory\n");
			goto fail1;
		}

		/*
		 * Create TX ring DMA stuffs
		 */
		error = bus_dma_tag_create(dma->parent_dtag,
				    BCM_ALIGN, 0,
				    BUS_SPACE_MAXADDR,
				    BUS_SPACE_MAXADDR,
				    NULL, NULL,
				    BCM_HDRSIZE(mac),
				    1,
				    BUS_SPACE_MAXSIZE_32BIT,
				    0,
				    NULL, NULL,
				    &dr->dr_txring_dtag);
		if (error) {
			device_printf(sc->sc_dev,
			    "can't create TX ring DMA tag: TODO frees\n");
			goto fail2;
		}

		for (i = 0; i < dr->dr_numslots; i += 2) {
			dr->getdesc(dr, i, &desc, &mt);

			mt->mt_txtype = BCM_DMADESC_METATYPE_HEADER;
			mt->mt_m = NULL;
			mt->mt_ni = NULL;
			mt->mt_islast = 0;
			error = bus_dmamap_create(dr->dr_txring_dtag, 0,
			    &mt->mt_dmap);
			if (error) {
				device_printf(sc->sc_dev,
				     "can't create RX buf DMA map\n");
				goto fail2;
			}

			dr->getdesc(dr, i + 1, &desc, &mt);

			mt->mt_txtype = BCM_DMADESC_METATYPE_BODY;
			mt->mt_m = NULL;
			mt->mt_ni = NULL;
			mt->mt_islast = 1;
			error = bus_dmamap_create(dma->txbuf_dtag, 0,
			    &mt->mt_dmap);
			if (error) {
				device_printf(sc->sc_dev,
				     "can't create RX buf DMA map\n");
				goto fail2;
			}
		}
	} else {
		error = bus_dmamap_create(dma->rxbuf_dtag, 0,
		    &dr->dr_spare_dmap);
		if (error) {
			device_printf(sc->sc_dev,
			    "can't create RX buf DMA map\n");
			goto out;		/* XXX wrong! */
		}

		for (i = 0; i < dr->dr_numslots; i++) {
			dr->getdesc(dr, i, &desc, &mt);

			error = bus_dmamap_create(dma->rxbuf_dtag, 0,
			    &mt->mt_dmap);
			if (error) {
				device_printf(sc->sc_dev,
				    "can't create RX buf DMA map\n");
				goto out;	/* XXX wrong! */
			}
			error = bcm_dma_newbuf(dr, desc, mt, 1);
			if (error) {
				device_printf(sc->sc_dev,
				    "failed to allocate RX buf\n");
				goto out;	/* XXX wrong! */
			}
		}

		bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap,
		    BUS_DMASYNC_PREWRITE);

		dr->dr_usedslot = dr->dr_numslots;
	}

      out:
	return (dr);

fail2:
	if (dr->dr_txhdr_cache != NULL) {
		contigfree(dr->dr_txhdr_cache,
		    (dr->dr_numslots / BCM_TX_SLOTS_PER_FRAME) *
		    BCM_MAXTXHDRSIZE, M_DEVBUF);
	}
fail1:
	free(dr->dr_meta, M_DEVBUF);
fail0:
	free(dr, M_DEVBUF);
	return (NULL);
}

static void
bcm_dma_ringfree(struct bcm_dma_ring **dr)
{

	if (dr == NULL)
		return;

	bcm_dma_free_descbufs(*dr);
	bcm_dma_free_ringmemory(*dr);

	if ((*dr)->dr_txhdr_cache != NULL) {
		contigfree((*dr)->dr_txhdr_cache,
		    ((*dr)->dr_numslots / BCM_TX_SLOTS_PER_FRAME) *
		    BCM_MAXTXHDRSIZE, M_DEVBUF);
	}
	free((*dr)->dr_meta, M_DEVBUF);
	free(*dr, M_DEVBUF);

	*dr = NULL;
}

static void
bcm_dma_32_getdesc(struct bcm_dma_ring *dr, int slot,
    struct bcm_dmadesc_generic **gdesc, struct bcm_dmadesc_meta **meta)
{
	struct bcm_dmadesc32 *desc;

	*meta = &(dr->dr_meta[slot]);
	desc = dr->dr_ring_descbase;
	desc = &(desc[slot]);

	*gdesc = (struct bcm_dmadesc_generic *)desc;
}

static void
bcm_dma_32_setdesc(struct bcm_dma_ring *dr,
    struct bcm_dmadesc_generic *desc, bus_addr_t dmaaddr, uint16_t bufsize,
    int start, int end, int irq)
{
	struct bcm_dmadesc32 *descbase = dr->dr_ring_descbase;
	struct bcm_softc *sc = dr->dr_mac->mac_sc;
	uint32_t addr, addrext, ctl;
	int slot;

	slot = (int)(&(desc->dma.dma32) - descbase);
	KASSERT(slot >= 0 && slot < dr->dr_numslots,
	    ("%s:%d: fail", __func__, __LINE__));

	addr = (uint32_t) (dmaaddr & ~SIBA_DMA_TRANSLATION_MASK);
	addrext = (uint32_t) (dmaaddr & SIBA_DMA_TRANSLATION_MASK) >> 30;
	addr |= siba_dma_translation(sc->sc_dev);
	ctl = bufsize & BCM_DMA32_DCTL_BYTECNT;
	if (slot == dr->dr_numslots - 1)
		ctl |= BCM_DMA32_DCTL_DTABLEEND;
	if (start)
		ctl |= BCM_DMA32_DCTL_FRAMESTART;
	if (end)
		ctl |= BCM_DMA32_DCTL_FRAMEEND;
	if (irq)
		ctl |= BCM_DMA32_DCTL_IRQ;
	ctl |= (addrext << BCM_DMA32_DCTL_ADDREXT_SHIFT)
	    & BCM_DMA32_DCTL_ADDREXT_MASK;

	desc->dma.dma32.control = htole32(ctl);
	desc->dma.dma32.address = htole32(addr);
}

static void
bcm_dma_32_start_transfer(struct bcm_dma_ring *dr, int slot)
{

	BCM_DMA_WRITE(dr, BCM_DMA32_TXINDEX,
	    (uint32_t)(slot * sizeof(struct bcm_dmadesc32)));
}

static void
bcm_dma_32_suspend(struct bcm_dma_ring *dr)
{

	BCM_DMA_WRITE(dr, BCM_DMA32_TXCTL,
	    BCM_DMA_READ(dr, BCM_DMA32_TXCTL) | BCM_DMA32_TXSUSPEND);
}

static void
bcm_dma_32_resume(struct bcm_dma_ring *dr)
{

	BCM_DMA_WRITE(dr, BCM_DMA32_TXCTL,
	    BCM_DMA_READ(dr, BCM_DMA32_TXCTL) & ~BCM_DMA32_TXSUSPEND);
}

static int
bcm_dma_32_get_curslot(struct bcm_dma_ring *dr)
{
	uint32_t val;

	val = BCM_DMA_READ(dr, BCM_DMA32_RXSTATUS);
	val &= BCM_DMA32_RXDPTR;

	return (val / sizeof(struct bcm_dmadesc32));
}

static void
bcm_dma_32_set_curslot(struct bcm_dma_ring *dr, int slot)
{

	BCM_DMA_WRITE(dr, BCM_DMA32_RXINDEX,
	    (uint32_t) (slot * sizeof(struct bcm_dmadesc32)));
}

static void
bcm_dma_64_getdesc(struct bcm_dma_ring *dr, int slot,
    struct bcm_dmadesc_generic **gdesc, struct bcm_dmadesc_meta **meta)
{
	struct bcm_dmadesc64 *desc;

	*meta = &(dr->dr_meta[slot]);
	desc = dr->dr_ring_descbase;
	desc = &(desc[slot]);

	*gdesc = (struct bcm_dmadesc_generic *)desc;
}

static void
bcm_dma_64_setdesc(struct bcm_dma_ring *dr,
    struct bcm_dmadesc_generic *desc, bus_addr_t dmaaddr, uint16_t bufsize,
    int start, int end, int irq)
{
	struct bcm_dmadesc64 *descbase = dr->dr_ring_descbase;
	struct bcm_softc *sc = dr->dr_mac->mac_sc;
	int slot;
	uint32_t ctl0 = 0, ctl1 = 0;
	uint32_t addrlo, addrhi;
	uint32_t addrext;

	slot = (int)(&(desc->dma.dma64) - descbase);
	KASSERT(slot >= 0 && slot < dr->dr_numslots,
	    ("%s:%d: fail", __func__, __LINE__));

	addrlo = (uint32_t) (dmaaddr & 0xffffffff);
	addrhi = (((uint64_t) dmaaddr >> 32) & ~SIBA_DMA_TRANSLATION_MASK);
	addrext = (((uint64_t) dmaaddr >> 32) & SIBA_DMA_TRANSLATION_MASK) >>
	    30;
	addrhi |= (siba_dma_translation(sc->sc_dev) << 1);
	if (slot == dr->dr_numslots - 1)
		ctl0 |= BCM_DMA64_DCTL0_DTABLEEND;
	if (start)
		ctl0 |= BCM_DMA64_DCTL0_FRAMESTART;
	if (end)
		ctl0 |= BCM_DMA64_DCTL0_FRAMEEND;
	if (irq)
		ctl0 |= BCM_DMA64_DCTL0_IRQ;
	ctl1 |= bufsize & BCM_DMA64_DCTL1_BYTECNT;
	ctl1 |= (addrext << BCM_DMA64_DCTL1_ADDREXT_SHIFT)
	    & BCM_DMA64_DCTL1_ADDREXT_MASK;

	desc->dma.dma64.control0 = htole32(ctl0);
	desc->dma.dma64.control1 = htole32(ctl1);
	desc->dma.dma64.address_low = htole32(addrlo);
	desc->dma.dma64.address_high = htole32(addrhi);
}

static void
bcm_dma_64_start_transfer(struct bcm_dma_ring *dr, int slot)
{

	BCM_DMA_WRITE(dr, BCM_DMA64_TXINDEX,
	    (uint32_t)(slot * sizeof(struct bcm_dmadesc64)));
}

static void
bcm_dma_64_suspend(struct bcm_dma_ring *dr)
{

	BCM_DMA_WRITE(dr, BCM_DMA64_TXCTL,
	    BCM_DMA_READ(dr, BCM_DMA64_TXCTL) | BCM_DMA64_TXSUSPEND);
}

static void
bcm_dma_64_resume(struct bcm_dma_ring *dr)
{

	BCM_DMA_WRITE(dr, BCM_DMA64_TXCTL,
	    BCM_DMA_READ(dr, BCM_DMA64_TXCTL) & ~BCM_DMA64_TXSUSPEND);
}

static int
bcm_dma_64_get_curslot(struct bcm_dma_ring *dr)
{
	uint32_t val;

	val = BCM_DMA_READ(dr, BCM_DMA64_RXSTATUS);
	val &= BCM_DMA64_RXSTATDPTR;

	return (val / sizeof(struct bcm_dmadesc64));
}

static void
bcm_dma_64_set_curslot(struct bcm_dma_ring *dr, int slot)
{

	BCM_DMA_WRITE(dr, BCM_DMA64_RXINDEX,
	    (uint32_t)(slot * sizeof(struct bcm_dmadesc64)));
}

static int
bcm_dma_allocringmemory(struct bcm_dma_ring *dr)
{
	struct bcm_mac *mac = dr->dr_mac;
	struct bcm_dma *dma = &mac->mac_method.dma;
	struct bcm_softc *sc = mac->mac_sc;
	int error;

	error = bus_dma_tag_create(dma->parent_dtag,
			    BCM_ALIGN, 0,
			    BUS_SPACE_MAXADDR,
			    BUS_SPACE_MAXADDR,
			    NULL, NULL,
			    BCM_DMA_RINGMEMSIZE,
			    1,
			    BUS_SPACE_MAXSIZE_32BIT,
			    0,
			    NULL, NULL,
			    &dr->dr_ring_dtag);
	if (error) {
		device_printf(sc->sc_dev,
		    "can't create TX ring DMA tag: TODO frees\n");
		return (-1);
	}

	error = bus_dmamem_alloc(dr->dr_ring_dtag,
	    &dr->dr_ring_descbase, BUS_DMA_WAITOK | BUS_DMA_ZERO,
	    &dr->dr_ring_dmap);
	if (error) {
		device_printf(sc->sc_dev,
		    "can't allocate DMA mem: TODO frees\n");
		return (-1);
	}
	error = bus_dmamap_load(dr->dr_ring_dtag, dr->dr_ring_dmap,
	    dr->dr_ring_descbase, BCM_DMA_RINGMEMSIZE,
	    bcm_dma_ring_addr, &dr->dr_ring_dmabase, BUS_DMA_NOWAIT);
	if (error) {
		device_printf(sc->sc_dev,
		    "can't load DMA mem: TODO free\n");
		return (-1);
	}

	return (0);
}

static void
bcm_dma_setup(struct bcm_dma_ring *dr)
{
	struct bcm_softc *sc = dr->dr_mac->mac_sc;
	uint64_t ring64;
	uint32_t addrext, ring32, value;
	uint32_t trans = siba_dma_translation(sc->sc_dev);

	if (dr->dr_tx) {
		dr->dr_curslot = -1;

		if (dr->dr_type == BCM_DMA_64BIT) {
			ring64 = (uint64_t)(dr->dr_ring_dmabase);
			addrext = ((ring64 >> 32) & SIBA_DMA_TRANSLATION_MASK)
			    >> 30;
			value = BCM_DMA64_TXENABLE;
			value |= (addrext << BCM_DMA64_TXADDREXT_SHIFT)
			    & BCM_DMA64_TXADDREXT_MASK;
			BCM_DMA_WRITE(dr, BCM_DMA64_TXCTL, value);
			BCM_DMA_WRITE(dr, BCM_DMA64_TXRINGLO,
			    (ring64 & 0xffffffff));
			BCM_DMA_WRITE(dr, BCM_DMA64_TXRINGHI,
			    ((ring64 >> 32) &
			    ~SIBA_DMA_TRANSLATION_MASK) | (trans << 1));
		} else {
			ring32 = (uint32_t)(dr->dr_ring_dmabase);
			addrext = (ring32 & SIBA_DMA_TRANSLATION_MASK) >> 30;
			value = BCM_DMA32_TXENABLE;
			value |= (addrext << BCM_DMA32_TXADDREXT_SHIFT)
			    & BCM_DMA32_TXADDREXT_MASK;
			BCM_DMA_WRITE(dr, BCM_DMA32_TXCTL, value);
			BCM_DMA_WRITE(dr, BCM_DMA32_TXRING,
			    (ring32 & ~SIBA_DMA_TRANSLATION_MASK) | trans);
		}
		return;
	}

	/*
	 * set for RX
	 */
	dr->dr_usedslot = dr->dr_numslots;

	if (dr->dr_type == BCM_DMA_64BIT) {
		ring64 = (uint64_t)(dr->dr_ring_dmabase);
		addrext = ((ring64 >> 32) & SIBA_DMA_TRANSLATION_MASK) >> 30;
		value = (dr->dr_frameoffset << BCM_DMA64_RXFROFF_SHIFT);
		value |= BCM_DMA64_RXENABLE;
		value |= (addrext << BCM_DMA64_RXADDREXT_SHIFT)
		    & BCM_DMA64_RXADDREXT_MASK;
		BCM_DMA_WRITE(dr, BCM_DMA64_RXCTL, value);
		BCM_DMA_WRITE(dr, BCM_DMA64_RXRINGLO, (ring64 & 0xffffffff));
		BCM_DMA_WRITE(dr, BCM_DMA64_RXRINGHI,
		    ((ring64 >> 32) & ~SIBA_DMA_TRANSLATION_MASK)
		    | (trans << 1));
		BCM_DMA_WRITE(dr, BCM_DMA64_RXINDEX, dr->dr_numslots *
		    sizeof(struct bcm_dmadesc64));
	} else {
		ring32 = (uint32_t)(dr->dr_ring_dmabase);
		addrext = (ring32 & SIBA_DMA_TRANSLATION_MASK) >> 30;
		value = (dr->dr_frameoffset << BCM_DMA32_RXFROFF_SHIFT);
		value |= BCM_DMA32_RXENABLE;
		value |= (addrext << BCM_DMA32_RXADDREXT_SHIFT)
		    & BCM_DMA32_RXADDREXT_MASK;
		BCM_DMA_WRITE(dr, BCM_DMA32_RXCTL, value);
		BCM_DMA_WRITE(dr, BCM_DMA32_RXRING,
		    (ring32 & ~SIBA_DMA_TRANSLATION_MASK) | trans);
		BCM_DMA_WRITE(dr, BCM_DMA32_RXINDEX, dr->dr_numslots *
		    sizeof(struct bcm_dmadesc32));
	}
}

static void
bcm_dma_free_ringmemory(struct bcm_dma_ring *dr)
{

	bus_dmamap_unload(dr->dr_ring_dtag, dr->dr_ring_dmap);
	bus_dmamem_free(dr->dr_ring_dtag, dr->dr_ring_descbase,
	    dr->dr_ring_dmap);
}

static void
bcm_dma_cleanup(struct bcm_dma_ring *dr)
{

	if (dr->dr_tx) {
		bcm_dma_tx_reset(dr->dr_mac, dr->dr_base, dr->dr_type);
		if (dr->dr_type == BCM_DMA_64BIT) {
			BCM_DMA_WRITE(dr, BCM_DMA64_TXRINGLO, 0);
			BCM_DMA_WRITE(dr, BCM_DMA64_TXRINGHI, 0);
		} else
			BCM_DMA_WRITE(dr, BCM_DMA32_TXRING, 0);
	} else {
		bcm_dma_rx_reset(dr->dr_mac, dr->dr_base, dr->dr_type);
		if (dr->dr_type == BCM_DMA_64BIT) {
			BCM_DMA_WRITE(dr, BCM_DMA64_RXRINGLO, 0);
			BCM_DMA_WRITE(dr, BCM_DMA64_RXRINGHI, 0);
		} else
			BCM_DMA_WRITE(dr, BCM_DMA32_RXRING, 0);
	}
}

static void
bcm_dma_free_descbufs(struct bcm_dma_ring *dr)
{
	struct bcm_dmadesc_generic *desc;
	struct bcm_dmadesc_meta *meta;
	struct bcm_mac *mac = dr->dr_mac;
	struct bcm_dma *dma = &mac->mac_method.dma;
	struct bcm_softc *sc = mac->mac_sc;
	int i;

	if (!dr->dr_usedslot)
		return;
	for (i = 0; i < dr->dr_numslots; i++) {
		dr->getdesc(dr, i, &desc, &meta);

		if (meta->mt_m == NULL) {
			if (!dr->dr_tx)
				device_printf(sc->sc_dev, "%s: not TX?\n",
				    __func__);
			continue;
		}
		if (dr->dr_tx) {
			if (meta->mt_txtype == BCM_DMADESC_METATYPE_HEADER)
				bus_dmamap_unload(dr->dr_txring_dtag,
				    meta->mt_dmap);
			else if (meta->mt_txtype == BCM_DMADESC_METATYPE_BODY)
				bus_dmamap_unload(dma->txbuf_dtag,
				    meta->mt_dmap);
		} else
			bus_dmamap_unload(dma->rxbuf_dtag, meta->mt_dmap);
		bcm_dma_free_descbuf(dr, meta);
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

static int
bcm_dma_rx_reset(struct bcm_mac *mac, uint16_t base,
    int type)
{
	struct bcm_softc *sc = mac->mac_sc;
	uint32_t value;
	int i;
	uint16_t offset;

	offset = (type == BCM_DMA_64BIT) ? BCM_DMA64_RXCTL : BCM_DMA32_RXCTL;
	BCM_WRITE_4(mac, base + offset, 0);
	for (i = 0; i < 10; i++) {
		offset = (type == BCM_DMA_64BIT) ? BCM_DMA64_RXSTATUS :
		    BCM_DMA32_RXSTATUS;
		value = BCM_READ_4(mac, base + offset);
		if (type == BCM_DMA_64BIT) {
			value &= BCM_DMA64_RXSTAT;
			if (value == BCM_DMA64_RXSTAT_DISABLED) {
				i = -1;
				break;
			}
		} else {
			value &= BCM_DMA32_RXSTATE;
			if (value == BCM_DMA32_RXSTAT_DISABLED) {
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

	return (0);
}

static void
bcm_dma_free_descbuf(struct bcm_dma_ring *dr,
    struct bcm_dmadesc_meta *meta)
{

	if (meta->mt_m != NULL) {
		m_freem(meta->mt_m);
		meta->mt_m = NULL;
	}
	if (meta->mt_ni != NULL) {
		ieee80211_free_node(meta->mt_ni);
		meta->mt_ni = NULL;
	}
}

static void
bcm_dma_set_redzone(struct bcm_dma_ring *dr, struct mbuf *m)
{
	struct bcm_rxhdr4 *rxhdr;
	unsigned char *frame;

	rxhdr = mtod(m, struct bcm_rxhdr4 *);
	rxhdr->frame_len = 0;

	KASSERT(dr->dr_rx_bufsize >= dr->dr_frameoffset +
	    sizeof(struct bcm_plcp6) + 2,
	    ("%s:%d: fail", __func__, __LINE__));
	frame = mtod(m, char *) + dr->dr_frameoffset;
	memset(frame, 0xff, sizeof(struct bcm_plcp6) + 2 /* padding */);
}

static uint8_t
bcm_dma_check_redzone(struct bcm_dma_ring *dr, struct mbuf *m)
{
	unsigned char *f = mtod(m, char *) + dr->dr_frameoffset;

	return ((f[0] & f[1] & f[2] & f[3] & f[4] & f[5] & f[6] & f[7])
	    == 0xff);
}

static int
bcm_dma_gettype(struct bcm_mac *mac)
{
	uint32_t tmp;
	uint16_t base;

	tmp = BCM_READ_4(mac, SIBA_TGSHIGH);
	if (tmp & SIBA_TGSHIGH_DMA64)
		return (BCM_DMA_64BIT);
	base = bcm_dma_base(0, 0);
	BCM_WRITE_4(mac, base + BCM_DMA32_TXCTL, BCM_DMA32_TXADDREXT_MASK);
	tmp = BCM_READ_4(mac, base + BCM_DMA32_TXCTL);
	if (tmp & BCM_DMA32_TXADDREXT_MASK)
		return (BCM_DMA_32BIT);

	return (BCM_DMA_30BIT);
}

static void
bcm_dma_ring_addr(void *arg, bus_dma_segment_t *seg, int nseg, int error)
{
	if (!error) {
		KASSERT(nseg == 1, ("too many segments(%d)\n", nseg));
		*((bus_addr_t *)arg) = seg->ds_addr;
	}
}

static void
bcm_dma_free(struct bcm_mac *mac)
{
	struct bcm_dma *dma;

	if ((mac->mac_flags & BCM_MAC_FLAG_DMA) == 0)
		return;
	dma = &mac->mac_method.dma;

	bcm_dma_ringfree(&dma->rx);
	bcm_dma_ringfree(&dma->wme[WME_AC_BK]);
	bcm_dma_ringfree(&dma->wme[WME_AC_BE]);
	bcm_dma_ringfree(&dma->wme[WME_AC_VI]);
	bcm_dma_ringfree(&dma->wme[WME_AC_VO]);
	bcm_dma_ringfree(&dma->mcast);
}

static void
bcm_dma_rx(struct bcm_dma_ring *dr)
{
	int slot, curslot;

	KASSERT(!dr->dr_tx, ("%s:%d: fail", __func__, __LINE__));
	curslot = dr->get_curslot(dr);
	KASSERT(curslot >= 0 && curslot < dr->dr_numslots,
	    ("%s:%d: fail", __func__, __LINE__));

	slot = dr->dr_curslot;
	for (; slot != curslot; slot = bcm_dma_nextslot(dr, slot))
		bcm_dma_rxeof(dr, &slot);

	bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap,
	    BUS_DMASYNC_PREWRITE);

	dr->set_curslot(dr, slot);
	dr->dr_curslot = slot;
}

static int
bcm_dma_freeslot(struct bcm_dma_ring *dr)
{
	BCM_ASSERT_LOCKED(dr->dr_mac->mac_sc);

	return (dr->dr_numslots - dr->dr_usedslot);
}

static int
bcm_dma_nextslot(struct bcm_dma_ring *dr, int slot)
{
	BCM_ASSERT_LOCKED(dr->dr_mac->mac_sc);

	KASSERT(slot >= -1 && slot <= dr->dr_numslots - 1,
	    ("%s:%d: fail", __func__, __LINE__));
	if (slot == dr->dr_numslots - 1)
		return (0);
	return (slot + 1);
}

static void
bcm_dma_rxeof(struct bcm_dma_ring *dr, int *slot)
{
	struct bcm_mac *mac = dr->dr_mac;
	struct bcm_softc *sc = mac->mac_sc;
	struct bcm_dma *dma = &mac->mac_method.dma;
	struct bcm_dmadesc_generic *desc;
	struct bcm_dmadesc_meta *meta;
	struct bcm_rxhdr4 *rxhdr;
	struct mbuf *m;
	uint32_t macstat;
	int32_t tmp;
	int cnt = 0;
	uint16_t len;

	dr->getdesc(dr, *slot, &desc, &meta);

	bus_dmamap_sync(dma->rxbuf_dtag, meta->mt_dmap, BUS_DMASYNC_POSTREAD);
	m = meta->mt_m;

	if (bcm_dma_newbuf(dr, desc, meta, 0)) {
		counter_u64_add(sc->sc_ic.ic_ierrors, 1);
		return;
	}

	rxhdr = mtod(m, struct bcm_rxhdr4 *);
	len = le16toh(rxhdr->frame_len);
	if (len <= 0) {
		counter_u64_add(sc->sc_ic.ic_ierrors, 1);
		return;
	}
	if (bcm_dma_check_redzone(dr, m)) {
		device_printf(sc->sc_dev, "redzone error.\n");
		bcm_dma_set_redzone(dr, m);
		bus_dmamap_sync(dma->rxbuf_dtag, meta->mt_dmap,
		    BUS_DMASYNC_PREWRITE);
		return;
	}
	if (len > dr->dr_rx_bufsize) {
		tmp = len;
		while (1) {
			dr->getdesc(dr, *slot, &desc, &meta);
			bcm_dma_set_redzone(dr, meta->mt_m);
			bus_dmamap_sync(dma->rxbuf_dtag, meta->mt_dmap,
			    BUS_DMASYNC_PREWRITE);
			*slot = bcm_dma_nextslot(dr, *slot);
			cnt++;
			tmp -= dr->dr_rx_bufsize;
			if (tmp <= 0)
				break;
		}
		device_printf(sc->sc_dev, "too small buffer "
		       "(len %u buffer %u dropped %d)\n",
		       len, dr->dr_rx_bufsize, cnt);
		return;
	}

	switch (mac->mac_fw.fw_hdr_format) {
	case BCM_FW_HDR_351:
	case BCM_FW_HDR_410:
		macstat = le32toh(rxhdr->ps4.r351.mac_status);
		break;
	case BCM_FW_HDR_598:
		macstat = le32toh(rxhdr->ps4.r598.mac_status);
		break;
	}

	if (macstat & BCM_RX_MAC_FCSERR) {
		if (!(mac->mac_sc->sc_filters & BCM_MACCTL_PASS_BADFCS)) {
			device_printf(sc->sc_dev, "RX drop\n");
			return;
		}
	}

	m->m_len = m->m_pkthdr.len = len + dr->dr_frameoffset;
	m_adj(m, dr->dr_frameoffset);

	bcm_rxeof(dr->dr_mac, m, rxhdr);
}

static int
bcm_dma_newbuf(struct bcm_dma_ring *dr, struct bcm_dmadesc_generic *desc,
    struct bcm_dmadesc_meta *meta, int init)
{
	struct bcm_mac *mac = dr->dr_mac;
	struct bcm_dma *dma = &mac->mac_method.dma;
	struct bcm_rxhdr4 *hdr;
	bus_dmamap_t map;
	bus_addr_t paddr;
	struct mbuf *m;
	int error;

	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL) {
		error = ENOBUFS;

		/*
		 * If the NIC is up and running, we need to:
		 * - Clear RX buffer's header.
		 * - Restore RX descriptor settings.
		 */
		if (init)
			return (error);
		else
			goto back;
	}
	m->m_len = m->m_pkthdr.len = MCLBYTES;

	bcm_dma_set_redzone(dr, m);

	/*
	 * Try to load RX buf into temporary DMA map
	 */
	error = bus_dmamap_load_mbuf(dma->rxbuf_dtag, dr->dr_spare_dmap, m,
	    bcm_dma_buf_addr, &paddr, BUS_DMA_NOWAIT);
	if (error) {
		m_freem(m);

		/*
		 * See the comment above
		 */
		if (init)
			return (error);
		else
			goto back;
	}

	if (!init)
		bus_dmamap_unload(dma->rxbuf_dtag, meta->mt_dmap);
	meta->mt_m = m;
	meta->mt_paddr = paddr;

	/*
	 * Swap RX buf's DMA map with the loaded temporary one
	 */
	map = meta->mt_dmap;
	meta->mt_dmap = dr->dr_spare_dmap;
	dr->dr_spare_dmap = map;

back:
	/*
	 * Clear RX buf header
	 */
	hdr = mtod(meta->mt_m, struct bcm_rxhdr4 *);
	bzero(hdr, sizeof(*hdr));
	bus_dmamap_sync(dma->rxbuf_dtag, meta->mt_dmap,
	    BUS_DMASYNC_PREWRITE);

	/*
	 * Setup RX buf descriptor
	 */
	dr->setdesc(dr, desc, meta->mt_paddr, meta->mt_m->m_len -
	    sizeof(*hdr), 0, 0, 0);
	return (error);
}

static void
bcm_dma_buf_addr(void *arg, bus_dma_segment_t *seg, int nseg,
		 bus_size_t mapsz __unused, int error)
{

	if (!error) {
		KASSERT(nseg == 1, ("too many segments(%d)\n", nseg));
		*((bus_addr_t *)arg) = seg->ds_addr;
	}
}

static int
bcm_dma_attach(struct bcm_mac *mac)
{
	struct bcm_dma *dma = &mac->mac_method.dma;
	struct bcm_softc *sc = mac->mac_sc;
	bus_addr_t lowaddr = 0;
	int error;

	if (siba_get_type(sc->sc_dev) == SIBA_TYPE_PCMCIA || bcm_usedma == 0)
		return (0);

	KASSERT(siba_get_revid(sc->sc_dev) >= 5, ("%s: fail", __func__));

	mac->mac_flags |= BCM_MAC_FLAG_DMA;

	dma->dmatype = bcm_dma_gettype(mac);
	if (dma->dmatype == BCM_DMA_30BIT)
		lowaddr = BCM_BUS_SPACE_MAXADDR_30BIT;
	else if (dma->dmatype == BCM_DMA_32BIT)
		lowaddr = BUS_SPACE_MAXADDR_32BIT;
	else
		lowaddr = BUS_SPACE_MAXADDR;

	/*
	 * Create top level DMA tag
	 */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->sc_dev),	/* parent */
			       BCM_ALIGN, 0,		/* alignment, bounds */
			       lowaddr,			/* lowaddr */
			       BUS_SPACE_MAXADDR,	/* highaddr */
			       NULL, NULL,		/* filter, filterarg */
			       BUS_SPACE_MAXSIZE,	/* maxsize */
			       BUS_SPACE_UNRESTRICTED,	/* nsegments */
			       BUS_SPACE_MAXSIZE,	/* maxsegsize */
			       0,			/* flags */
			       NULL, NULL,		/* lockfunc, lockarg */
			       &dma->parent_dtag);
	if (error) {
		device_printf(sc->sc_dev, "can't create parent DMA tag\n");
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
		device_printf(sc->sc_dev, "can't create mbuf DMA tag\n");
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
		device_printf(sc->sc_dev, "can't create mbuf DMA tag\n");
		goto fail1;
	}

	dma->wme[WME_AC_BK] = bcm_dma_ringsetup(mac, 0, 1, dma->dmatype);
	if (!dma->wme[WME_AC_BK])
		goto fail2;

	dma->wme[WME_AC_BE] = bcm_dma_ringsetup(mac, 1, 1, dma->dmatype);
	if (!dma->wme[WME_AC_BE])
		goto fail3;

	dma->wme[WME_AC_VI] = bcm_dma_ringsetup(mac, 2, 1, dma->dmatype);
	if (!dma->wme[WME_AC_VI])
		goto fail4;

	dma->wme[WME_AC_VO] = bcm_dma_ringsetup(mac, 3, 1, dma->dmatype);
	if (!dma->wme[WME_AC_VO])
		goto fail5;

	dma->mcast = bcm_dma_ringsetup(mac, 4, 1, dma->dmatype);
	if (!dma->mcast)
		goto fail6;
	dma->rx = bcm_dma_ringsetup(mac, 0, 0, dma->dmatype);
	if (!dma->rx)
		goto fail7;

	return (error);

fail7:	bcm_dma_ringfree(&dma->mcast);
fail6:	bcm_dma_ringfree(&dma->wme[WME_AC_VO]);
fail5:	bcm_dma_ringfree(&dma->wme[WME_AC_VI]);
fail4:	bcm_dma_ringfree(&dma->wme[WME_AC_BE]);
fail3:	bcm_dma_ringfree(&dma->wme[WME_AC_BK]);
fail2:	bus_dma_tag_destroy(dma->txbuf_dtag);
fail1:	bus_dma_tag_destroy(dma->rxbuf_dtag);
fail0:	bus_dma_tag_destroy(dma->parent_dtag);
	return (error);
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
bcm_dma_stop(struct bcm_mac *mac)
{
	struct bcm_dma *dma;

	if ((mac->mac_flags & BCM_MAC_FLAG_DMA) == 0)
		return;
	dma = &mac->mac_method.dma;

	bcm_dma_ringstop(&dma->rx);
	bcm_dma_ringstop(&dma->wme[WME_AC_BK]);
	bcm_dma_ringstop(&dma->wme[WME_AC_BE]);
	bcm_dma_ringstop(&dma->wme[WME_AC_VI]);
	bcm_dma_ringstop(&dma->wme[WME_AC_VO]);
	bcm_dma_ringstop(&dma->mcast);
}

static void
bcm_dma_ringstop(struct bcm_dma_ring **dr)
{

	if (dr == NULL)
		return;

	bcm_dma_cleanup(*dr);
}

