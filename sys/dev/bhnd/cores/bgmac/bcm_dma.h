/*
 * bcm_dma.h
 *
 *  Created on: May 23, 2016
 *      Author: mizhka
 */

#ifndef SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_H_
#define SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_H_

#define	BCM_DMA_30BIT			30
#define	BCM_DMA_32BIT			32
#define	BCM_DMA_64BIT			64

struct bcm_dmadesc_meta {
	bus_dmamap_t			mt_dmap;
	bus_addr_t			mt_paddr;
	struct mbuf			*mt_m;
	struct ieee80211_node		*mt_ni;
	uint8_t				mt_txtype;
#define	BCM_DMADESC_METATYPE_HEADER	0
#define	BCM_DMADESC_METATYPE_BODY	1
	uint8_t				mt_islast;
};

#define	BCM_DMAINTR_FATALMASK	\
	((1 << 10) | (1 << 11) | (1 << 12) | (1 << 14) | (1 << 15))
#define	BCM_DMAINTR_NONFATALMASK	(1 << 13)
#define	BCM_DMAINTR_RX_DONE		(1 << 16)

#define	BCM_DMA32_DCTL_BYTECNT		0x00001fff
#define	BCM_DMA32_DCTL_ADDREXT_MASK	0x00030000
#define	BCM_DMA32_DCTL_ADDREXT_SHIFT	16
#define	BCM_DMA32_DCTL_DTABLEEND	0x10000000
#define	BCM_DMA32_DCTL_IRQ		0x20000000
#define	BCM_DMA32_DCTL_FRAMEEND		0x40000000
#define	BCM_DMA32_DCTL_FRAMESTART	0x80000000
struct bcm_dmadesc32 {
	uint32_t			control;
	uint32_t			address;
} __packed;

#define	BCM_DMA64_DCTL0_DTABLEEND	0x10000000
#define	BCM_DMA64_DCTL0_IRQ		0x20000000
#define	BCM_DMA64_DCTL0_FRAMEEND	0x40000000
#define	BCM_DMA64_DCTL0_FRAMESTART	0x80000000
#define	BCM_DMA64_DCTL1_BYTECNT		0x00001fff
#define	BCM_DMA64_DCTL1_ADDREXT_MASK	0x00030000
#define	BCM_DMA64_DCTL1_ADDREXT_SHIFT	16
struct bcm_dmadesc64 {
	uint32_t			control0;
	uint32_t			control1;
	uint32_t			address_low;
	uint32_t			address_high;
} __packed;

struct bcm_dmadesc_generic {
	union {
		struct bcm_dmadesc32 dma32;
		struct bcm_dmadesc64 dma64;
	} __packed dma;
} __packed;

struct bcm_dma_ring;

struct bcm_dma_ring {
	struct bcm_mac			*dr_mac;
	const struct bcm_dma_ops	*dr_ops;
	struct bcm_dmadesc_meta		*dr_meta;
	void				*dr_txhdr_cache;
	bus_dma_tag_t			dr_ring_dtag;
	bus_dma_tag_t			dr_txring_dtag;
	bus_dmamap_t			dr_spare_dmap; /* only for RX */
	bus_dmamap_t			dr_ring_dmap;
	bus_addr_t			dr_txring_paddr;
	void				*dr_ring_descbase;
	bus_addr_t			dr_ring_dmabase;
	int				dr_numslots;
	int				dr_usedslot;
	int				dr_curslot;
	uint32_t			dr_frameoffset;
	uint16_t			dr_rx_bufsize;
	uint16_t			dr_base;
	int				dr_index;
	uint8_t				dr_tx;
	uint8_t				dr_stop;
	int				dr_type;

	void				(*getdesc)(struct bcm_dma_ring *,
					    int, struct bcm_dmadesc_generic **,
					    struct bcm_dmadesc_meta **);
	void				(*setdesc)(struct bcm_dma_ring *,
					    struct bcm_dmadesc_generic *,
					    bus_addr_t, uint16_t, int, int,
					    int);
	void				(*start_transfer)(struct bcm_dma_ring *,
					    int);
	void				(*suspend)(struct bcm_dma_ring *);
	void				(*resume)(struct bcm_dma_ring *);
	int				(*get_curslot)(struct bcm_dma_ring *);
	void				(*set_curslot)(struct bcm_dma_ring *,
					    int);
};

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


#endif /* SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_H_ */
