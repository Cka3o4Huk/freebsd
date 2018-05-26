# $FreeBSD$

.PATH: ${SRCTOP}/sys/dev/dwc/
#.PATH: ${SRCTOP}/sys/arm/allwinner/

KMOD=	dwc
SRCS=	if_dwc.c if_dwc_if.c
SRCS+=  miibus_if.h 
#SRCS+=	aw_if_dwc.c
SRCS+=  device_if.h bus_if.h if_dwc_if.h gpio_if.h mdio_if.h
#SRCS+=	clknode_if.h regnode_if.h

.include <bsd.kmod.mk>
