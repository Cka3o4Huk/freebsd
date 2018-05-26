# $FreeBSD$

.PATH: ${SRCTOP}/sys/dev/etherswitch

KMOD=	etherswitch
SRCS=	etherswitch.c etherswitch_if.c
SRCS+=  etherswitch_if.h
SRCS+=  miibus_if.h mdio_if.h

.include <bsd.kmod.mk>
