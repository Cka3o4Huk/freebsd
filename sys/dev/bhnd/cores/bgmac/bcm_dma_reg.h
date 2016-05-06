/*
 * bcm_dmareg.h
 *
 *  Created on: May 24, 2016
 *      Author: mizhka
 */

#ifndef SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_REG_H_
#define SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_REG_H_

#define BCM_DMA_SHIFT(val, to)		((val << to ## _SHIFT) & to ## _MASK)

#define	BCM_DMA_CTL				0x00
#define		BCM_DMA_CTL_ENABLE		0x00000001
#define		BCM_DMA_CTL_SUSPEND		0x00000002
#define		BCM_DMA_CTL_RXFROFF_MASK	0x000000FE
#define		BCM_DMA_CTL_RXFROFF_SHIFT	1
#define		BCM_DMA_CTL_RXDIRECTFIFO	0x00000100
#define		BCM_DMA_CTL_OVERFLOWCONTINUE	0x00000400
#define		BCM_DMA_CTL_PARITYDISABLE	0x00000800
#define		BCM_DMA_CTL_ADDREXT_MASK	0x00030000
#define		BCM_DMA_CTL_ADDREXT_SHIFT	16

#define BCM_DMA_ADDR_MASK		0x3FFFFFFF

#define	BCM_DMA32_RING			0x04
#define	BCM_DMA32_INDEX			0x08
#define	BCM_DMA32_STATUS		0x0c

#define	BCM_DMA64_INDEX			0x04
#define	BCM_DMA64_RINGLO		0x08
#define	BCM_DMA64_RINGHI		0x0c
#define	BCM_DMA64_STATUS		0x10

#define	BCM_DMA32_STATE			0x0000f000
#define	BCM_DMA32_STATE_SHIFT		12
#define	BCM_DMA64_STATE			0xf0000000
#define	BCM_DMA64_STATE_SHIFT		28

#define	BCM_DMA_STAT_DISABLED		0
#define	BCM_DMA_STAT_IDLEWAIT		2
#define	BCM_DMA_STAT_STOPPED		3

#define	BCM_DMA32_RXBASE		0x10
#define	BCM_DMA64_RXBASE		0x20

#define	BCM_DMA32_RXDPTR		0x00000fff
/* XXX: was 1fff, but gmac sets this bit after first cycle and doesn't use it for pointer */
#define	BCM_DMA64_RXSTATDPTR		0x00000fff


#endif /* SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_REG_H_ */
