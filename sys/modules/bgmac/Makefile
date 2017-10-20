# $FreeBSD$

.PATH:	${SRCTOP}/sys/mips/broadcom/bgmac

KMOD=	bgmac
SRCS=	bcm_dma_ops.c bcm_dma_ring.c bcm_dma_rx.c bcm_dma_tx.c bcm_dma.c bgmac_ddb.c bgmac_tx.c bgmac.c

SRCS+=	miidevs.h
#SRCS+=	device_if.h bus_if.h
#SRCS+=	bhnd_bus_if.h bhnd_erom_if.h bhnd_pmu_if.h

.include <bsd.kmod.mk>
