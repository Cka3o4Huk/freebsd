/*
 * bcm_dma.h
 *
 *  Created on: May 23, 2016
 *      Author: mizhka
 */

#ifndef SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_H_
#define SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_H_

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>

#define	BCM_DMA_30BIT			30
#define	BCM_DMA_32BIT			32
#define	BCM_DMA_64BIT			64


#define	BCM_DMAINTR_FATALMASK	\
	((1 << 10) | (1 << 11) | (1 << 12) | (1 << 14) | (1 << 15))
#define	BCM_DMAINTR_NONFATALMASK	(1 << 13)
#define	BCM_DMAINTR_RX_DONE		(1 << 16)

#define	BCM_DMA_BIT_MASK(n)		(((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))

struct bcm_dma {
	int				dmatype;
	bus_dma_tag_t			parent_dtag;
	bus_dma_tag_t			rxbuf_dtag;
	bus_dma_tag_t			txbuf_dtag;

	struct bcm_dma_ring		*wme[5];
	struct bcm_dma_ring		*mcast;
	struct bcm_dma_ring		*rx;
	uint64_t			lastseq;	/* XXX FIXME */
};

#define BCM_FRAME_OFFSET		0x1e

struct bcm_rx_header {
	uint16_t len;
	uint16_t flags;
};

struct bwn_plcp6 {
	union {
		uint32_t		data;
		uint8_t			raw[6];
	} __packed o;
} __packed;

#define BCM_ASSERT_LOCKED(x)
#define BCM_DMA_ALIGN			0x1000

#define	BCM_DMA_RINGMEMSIZE		PAGE_SIZE

#define	BCM_TXRING_SLOTS		64
#define	BCM_RXRING_SLOTS		64

/* 32-bit DMA */

#define	BCM_DMA32_BASE_RX_SHIFT		0x010
#define	BCM_DMA32_BASE0			0x200
#define	BCM_DMA32_BASE1			0x220
#define	BCM_DMA32_BASE2			0x240
#define	BCM_DMA32_BASE3			0x260
#define	BCM_DMA32_BASE4			0x280
#define	BCM_DMA32_BASE5			0x2a0
/* 64-bit DMA */
#define	BCM_DMA64_BASE_RX_SHIFT		0x020
#define	BCM_DMA64_BASE0			0x200
#define	BCM_DMA64_BASE1			0x240
#define	BCM_DMA64_BASE2			0x280
#define	BCM_DMA64_BASE3			0x2c0
#define	BCM_DMA64_BASE4			0x300
#define	BCM_DMA64_BASE5			0x340

#define	N(a)			(sizeof(a) / sizeof(a[0]))

struct bcm_dma_ring;
struct bcm_dmadesc_generic;
struct bcm_dmadesc_meta;

void		bcm_dma_rx(struct bcm_dma_ring *dr);

int		bcm_dma_attach(device_t dev, struct resource *res,
		    struct bcm_dma *dma);
uint16_t	bcm_dma_base(int, int, int);
int		bcm_dma_rx_newbuf(struct bcm_dma_ring *dr,
		    struct bcm_dmadesc_generic *desc,
		    struct bcm_dmadesc_meta *meta, int init);
int		bcm_dma_rx_reset(struct bcm_dma_ring* ring,
		    uint16_t base, int type);

void		bcm_dmamap_callback(void *arg, bus_dma_segment_t *seg,
		    int nseg, int error);
void		bcm_dmamap_callback_mbuf(void *arg, bus_dma_segment_t *seg,
		    int nseg, bus_size_t mapsz __unused, int error);



#endif /* SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_H_ */
