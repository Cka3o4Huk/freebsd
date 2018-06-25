# $FreeBSD$

.PATH: ${SRCTOP}/sys/arm/allwinner

KMOD=	aw_dwc
SRCS=	aw_if_dwc.c

SRCS+=	\
	bus_if.h \
	clknode_if.h \
	device_if.h \
	gpio_if.h \
	if_dwc_if.h \
	mdio_if.h \
	miibus_if.h \
	regnode_if.h \

.include <bsd.kmod.mk>
