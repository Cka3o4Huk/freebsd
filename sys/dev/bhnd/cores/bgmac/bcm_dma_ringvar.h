/*
 * bcm_dmaring.h
 *
 *  Created on: May 24, 2016
 *      Author: mizhka
 */

#ifndef SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_RINGVAR_H_
#define SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_RINGVAR_H_

#include "bcm_dma_desc.h"
#include "bcm_dma.h"

#define	BCM_DMA_WRITE(ring, offset, value)				\
	bus_write_4(ring->res, ring->dr_base + offset, value)
#define	BCM_DMA_READ(ring, offset)					\
	bus_read_4(ring->res, ring->dr_base + offset)

struct bcm_dma_ring;

struct bcm_dma_ring {
	struct resource			*res;
	struct bcm_dma			*dma;
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
	uint8_t				dr_is_tx; 	/* 1 - TX; 0 - RX */
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

/*
 * Used in bcm_dma.c
 */
struct bcm_dma_ring*	bcm_dma_ring_setup(struct bcm_dma *dma,
			    struct resource *res,
			    int ctl_index,
			    int for_tx,
			    int type);
void			bcm_dma_ringload(struct bcm_dma_ring *dr);
void			bcm_dma_ringstop(struct bcm_dma_ring *dr);
void			bcm_dma_ring_free(struct bcm_dma_ring **dr);
int			bcm_dma_ring_alloc(struct bcm_dma *dma,
			    struct bcm_dma_ring *dr);
int			bcm_dmaring_get_freeslot(struct bcm_dma_ring *dr);
int			bcm_dmaring_get_nextslot(struct bcm_dma_ring *dr,
			    int slot);

#endif /* SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_RINGVAR_H_ */
