/*
 * bcm_dmaops.h
 *
 *  Created on: May 24, 2016
 *      Author: mizhka
 */

#ifndef SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_OPS_H_
#define SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_OPS_H_

struct bcm_dma_ring;

void	bcm_dma_suspend(struct bcm_dma_ring *);
void	bcm_dma_resume(struct bcm_dma_ring *);

void	bcm_dma_32_getdesc(struct bcm_dma_ring *, int,
	    struct bcm_dmadesc_generic **, struct bcm_dmadesc_meta **);
void	bcm_dma_32_setdesc(struct bcm_dma_ring *, struct bcm_dmadesc_generic *,
	    bus_addr_t, uint16_t, int, int, int);

void	bcm_dma_32_start_transfer(struct bcm_dma_ring *, int);
void	bcm_dma_32_resume(struct bcm_dma_ring *);
int	bcm_dma_32_get_curslot(struct bcm_dma_ring *);
void	bcm_dma_32_set_curslot(struct bcm_dma_ring *, int);

void	bcm_dma_64_getdesc(struct bcm_dma_ring *,
		    int, struct bcm_dmadesc_generic **,
		    struct bcm_dmadesc_meta **);
void	bcm_dma_64_setdesc(struct bcm_dma_ring *,
		    struct bcm_dmadesc_generic *, bus_addr_t, uint16_t, int,
		    int, int);
void	bcm_dma_64_start_transfer(struct bcm_dma_ring *, int);
int	bcm_dma_64_get_curslot(struct bcm_dma_ring *);
void	bcm_dma_64_set_curslot(struct bcm_dma_ring *, int);

#endif /* SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_OPS_H_ */
