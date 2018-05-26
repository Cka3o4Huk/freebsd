# $FreeBSD$

.PATH: ${SRCTOP}/sys/arm/allwinner/

KMOD=	a20_dwc
SRCS=	aw_if_dwc.c
SRCS+=  miibus_if.h 
SRCS+=  device_if.h bus_if.h if_dwc_if.h gpio_if.h mdio_if.h
SRCS+=	clknode_if.h regnode_if.h

.include <bsd.kmod.mk>
