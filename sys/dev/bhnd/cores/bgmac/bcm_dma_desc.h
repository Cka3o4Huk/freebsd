/*
 * bcm_dmadesc.h
 *
 *  Created on: May 24, 2016
 *      Author: mizhka
 */

#ifndef SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_DESC_H_
#define SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_DESC_H_

struct bcm_dmadesc_meta {
	bus_dmamap_t			mt_dmap;
	bus_addr_t			mt_paddr;
	struct mbuf			*mt_m;
	//struct ieee80211_node		*mt_ni;
//	uint8_t				mt_txtype;
#define	BCM_DMADESC_METATYPE_HEADER	0
#define	BCM_DMADESC_METATYPE_BODY	1
	uint8_t				mt_islast;
};

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

#define	BCM_DMADESC_DUMP(_desc)							\
	BHND_DEBUG("\tctl0=0x%x\n\tctl1=0x%x\n\taddrlow=0x%x\n\taddrhi=0x%x\n", \
	    (_desc)->dma.dma64.control0, (_desc)->dma.dma64.control1,		\
	    (_desc)->dma.dma64.address_low, (_desc)->dma.dma64.address_high);

#endif /* SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_DESC_H_ */
