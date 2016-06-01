/*
 * bcm_dma_rx.c
 *
 *  Created on: May 24, 2016
 *      Author: mizhka
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/errno.h>
#include <sys/endian.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include "bcm_dma_reg.h"
#include "bcm_dma_desc.h"
#include "bcm_dma_ops.h"
#include "bcm_dma_ringvar.h"

#define	BHND_LOGGING	BHND_INFO_LEVEL
#include <dev/bhnd/bhnd_debug.h>

static void	bcm_dma_rxeof(struct bcm_dma *dma, struct bcm_dma_ring *dr,
		    int *slot);
static uint8_t	bcm_dma_check_redzone(struct bcm_dma_ring *dr, struct mbuf *m);
static void	bcm_dma_set_redzone(struct bcm_dma_ring *dr, struct mbuf *m);

void
bcm_dma_rx(struct bcm_dma_ring *dr)
{
	int slot, curslot;

	/* Check if ring is RX */
	KASSERT(!dr->dr_is_tx, ("%s:%d: fail", __func__, __LINE__));
	curslot = dr->get_curslot(dr);
	BHND_DEBUG("hw rx slot: %d", curslot);
	KASSERT(curslot >= 0 && curslot < dr->dr_numslots,
	    ("%s:%d: fail", __func__, __LINE__));

	slot = dr->dr_curslot;
	BHND_DEBUG("sw rx slot: %d", slot);
	for (; slot != curslot; slot = bcm_dmaring_get_nextslot(dr, slot)) {
		BHND_DEBUG("processing slot: %d", slot);
		bcm_dma_rxeof(dr->dma, dr, &slot);
	}

	bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap,
	    BUS_DMASYNC_PREWRITE);

	dr->set_curslot(dr, slot);
	dr->dr_curslot = slot;
}

static void
bcm_dma_rxeof(struct bcm_dma *dma, struct bcm_dma_ring *dr, int *slot)
{
	struct bcm_dmadesc_generic 	*desc;
	struct bcm_dmadesc_meta 	*meta;
	struct bcm_rx_header		*rxhdr;
	struct mbuf 			*m;
	device_t			 dev;
	//uint32_t 			 macstat;
	int32_t 			 tmp;
	int 				 cnt = 0;
	uint16_t 			 len;

	dev = rman_get_device(dr->res);
	BHND_DEBUG_DEV(dev, "RECEIVED FRAME");

	dr->getdesc(dr, *slot, &desc, &meta);

	bus_dmamap_sync(dma->rxbuf_dtag, meta->mt_dmap, BUS_DMASYNC_POSTREAD);
	m = meta->mt_m;

	BHND_TRACE_DEV(dev, "received mbuf: %p", m);

	/*
	 * Create new buffer and put it into ring instead of dirty
	 */
	if (bcm_dma_rx_newbuf(dr, desc, meta, 0)) {
		/*
		 * TODO: add counters
		 */
		//counter_u64_add(sc->sc_ic.ic_ierrors, 1);
		device_printf(dev, "error on bcm_dma_rx_newbuf\n");
		return;
	}

	/*
	 * Process dirty received buffer
	 */
	rxhdr = mtod(m, struct bcm_rx_header *);
	len = le16toh(rxhdr->len);
	BHND_TRACE_DEV(dev, "len: %d", len);
#define HEXDUMP(_buf, _len) do { \
  { \
        size_t __tmp; \
        const char *__buf = (const char *)_buf; \
        for (__tmp = 0; __tmp < _len; __tmp++) \
                printf("%02hhx ", *__buf++); \
    printf("\n"); \
  } \
} while(0)

	//HEXDUMP(rxhdr,0x80);
#undef HEXDUMP
	if (len <= 0) {
		/*
		 * TODO: add counters
		 */
		//counter_u64_add(sc->sc_ic.ic_ierrors, 1);
		device_printf(dev, "len < 0\n");
		return;
	}
	if (bcm_dma_check_redzone(dr, m)) {
		device_printf(dev, "redzone error.\n");
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
			*slot = bcm_dmaring_get_nextslot(dr, *slot);
			cnt++;
			tmp -= dr->dr_rx_bufsize;
			if (tmp <= 0)
				break;
		}
		device_printf(dev, "too small buffer (len %u buf %u drop %d)\n",
		       len, dr->dr_rx_bufsize, cnt);
		return;
	}

	/*
	 * TODO: 802.11 code, uncomment it
	 */
//	switch (mac->mac_fw.fw_hdr_format) {
//	case BCM_FW_HDR_351:
//	case BCM_FW_HDR_410:
//		macstat = le32toh(rxhdr->ps4.r351.mac_status);
//		break;
//	case BCM_FW_HDR_598:
//		macstat = le32toh(rxhdr->ps4.r598.mac_status);
//		break;
//	}

//	if (macstat & BCM_RX_MAC_FCSERR) {
//		if (!(mac->mac_sc->sc_filters & BCM_MACCTL_PASS_BADFCS)) {
//			device_printf(sc->sc_dev, "RX drop\n");
//			return;
//		}
//	}

	m->m_len = m->m_pkthdr.len = len + dr->dr_frameoffset;
	m_adj(m, dr->dr_frameoffset);

	BHND_TRACE_DEV(dev, "success; calling MAC rxeof");
	/*
	 * Callback to MAC level rxeof
	 */
	bgmac_rxeof(rman_get_device(dr->res), m, rxhdr);
}


int
bcm_dma_rx_newbuf(struct bcm_dma_ring *dr, struct bcm_dmadesc_generic *desc,
    struct bcm_dmadesc_meta *meta, int init)
{
	//struct bcm_mac		*mac = dr->dr_mac;
	/*
	 * TODO: init bcm_dma *dma
	 */
	struct bcm_dma		*dma; //= &mac->mac_method.dma;
	struct bcm_rx_header	*hdr;
	struct mbuf		*m;
	bus_dmamap_t		 map;
	bus_addr_t		 paddr;
	int			 error;

	KASSERT(dr != NULL, ("ring is not specified"));
	dma = dr->dma;
	/*
	 * Get new mbuf
	 */
	if (!init)
		BHND_TRACE("allocate new mbuf cluster");
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
	if (0)
		BHND_DEBUG("load new mbuf[%p]: %p - %p - %p\n",
				dma,
				dma->rxbuf_dtag,
				dr->dr_spare_dmap,
				m);

	KASSERT(dma->rxbuf_dtag != NULL, ("rxbuf_dtag isn't initialized"));
	error = bus_dmamap_load_mbuf(dma->rxbuf_dtag, dr->dr_spare_dmap, m,
	    bcm_dmamap_callback_mbuf, &paddr, BUS_DMA_NOWAIT);
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


	/*
	 *  Clear RX buf header
	 */
back:
	hdr = mtod(meta->mt_m, struct bcm_rx_header *);
	bzero(hdr, sizeof(struct bcm_rx_header));
	bus_dmamap_sync(dma->rxbuf_dtag, meta->mt_dmap,
	    BUS_DMASYNC_PREWRITE);

	/*
	 * Setup RX buf descriptor
	 */
	dr->setdesc(dr, desc, meta->mt_paddr,
	    meta->mt_m->m_len - sizeof(struct bcm_rx_header), 0, 0, 0);
#if 0
	/*TODO: add debug configuration bits */
	BCM_DMADESC_DUMP(desc);
#endif
	return (error);
}

static void
bcm_dma_set_redzone(struct bcm_dma_ring *dr, struct mbuf *m)
{
	struct bcm_rx_header	*rxhdr;
	unsigned char		*frame;

	rxhdr = mtod(m, struct bcm_rx_header *);
	rxhdr->len = 0;
	rxhdr->flags = 0;

	/*
	 * TODO: uncomment it
	 */

//	KASSERT(dr->dr_rx_bufsize >= dr->dr_frameoffset +
//	    sizeof(struct bcm_plcp6) + 2,
//	    ("%s:%d: fail", __func__, __LINE__));
//
	frame = mtod(m, char *) + dr->dr_frameoffset;
	/* TODO: use M_CLBYTES macros here instead of 0x7d0 */
	memset(frame, 0xff, 0x7d0);
}

static uint8_t
bcm_dma_check_redzone(struct bcm_dma_ring *dr, struct mbuf *m)
{
	unsigned char *f = mtod(m, char *) + dr->dr_frameoffset;

	return ((f[0] & f[1] & f[2] & f[3] & f[4] & f[5] & f[6] & f[7])
	    == 0xff);
}

int
bcm_dma_rx_reset(struct bcm_dma_ring* ring, uint16_t base, int type)
{
	uint32_t		 value;
	int 			 i;
	uint16_t		 offset;

	BCM_DMA_WRITE(ring, BCM_DMA_CTL, 0);
	for (i = 0; i < 10; i++) {
		offset = (type == BCM_DMA_64BIT) ? BCM_DMA64_STATUS :
		    BCM_DMA32_STATUS;

		value = BCM_DMA_READ(ring, offset);
		value &= (type == BCM_DMA_64BIT) ? BCM_DMA64_STATE :
			     BCM_DMA32_STATE;

		if (value == BCM_DMA_STAT_DISABLED) {
			i = -1;
			break;
		}
		DELAY(1000);
	}
	if (i != -1) {
		device_printf(rman_get_device(ring->res),
		    "%s: timed out\n", __func__);
		return (ENODEV);
	}

	return (0);
}

//static void
//bcm_dma_rxdirectfifo(struct bcm_dma_ring* ring, int idx, uint8_t enable)
//{
//	uint32_t ctl;
//	int type;
//	uint16_t base;
//
//	type = bcm_dma_mask2type(bcm_dma_mask(mac));
//	base = bcm_dma_base(type, idx);
//	if (type == BCM_DMA_64BIT) {
//		ctl = BCM_READ_4(mac, base + BCM_DMA_CTL);
//		ctl &= ~BCM_DMA64_RXDIRECTFIFO;
//		if (enable)
//			ctl |= BCM_DMA64_RXDIRECTFIFO;
//		BCM_WRITE_4(mac, base + BCM_DMA64_RXCTL, ctl);
//	} else {
//		ctl = BCM_READ_4(mac, base + BCM_DMA32_RXCTL);
//		ctl &= ~BCM_DMA32_RXDIRECTFIFO;
//		if (enable)
//			ctl |= BCM_DMA32_RXDIRECTFIFO;
//		BCM_WRITE_4(mac, base + BCM_DMA32_RXCTL, ctl);
//	}
//}
//


