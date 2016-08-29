/*-
 * Copyright (c) 2011 Aleksandr Rybalko.
 * Copyright (c) 2016 Michael Zhilin.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _B53_VAR_H_
#define _B53_VAR_H_

#include <sys/param.h>
#include <sys/malloc.h>
#include <sys/callout.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>

#include <dev/etherswitch/etherswitch.h>
#include <dev/mii/mii.h>

#define	B53_NUM_PHYS	9
#define	B53_DEF_VLANID	1
#define B53_DEF_MASK	0x11e

MALLOC_DECLARE(M_BCMSWITCH);

struct b53_softc;
struct b53_hal;

#define	B53HALSIZE		6

typedef void (*voidfunctype) (void);

struct b53_functions {
	union{
		struct {
			int (* reset) (struct b53_softc *sc);
			/* VLAN functions */
			int (* vlan_enable) (struct b53_softc *sc, int on);
			int (* vlan_get_pvid) (struct b53_softc *sc, int port,
			    int *pvid);
			int (* vlan_set_pvid) (struct b53_softc *sc, int port,
			    int pvid);
			int (* vlan_get_vlan_group) (struct b53_softc *sc,
			    int vlan_group, int *vlan_id, int *members,
			    int *untagged, int *forward_id);
			int (* vlan_set_vlan_group) (struct b53_softc *sc,
			    int vlan_group, int vlan_id, int members,
			    int untagged, int forward_id);
		};
		voidfunctype func[B53HALSIZE];
	};
};

struct b53_hal;

struct b53_hal {
	struct b53_hal		*parent;
	struct b53_functions	*own;
};

struct b53_softc {
	struct mtx	 sc_mtx;		/* serialize access to softc */
	device_t	 sc_dev;
	device_t	 sc_parent;
	int		 media;		/* cpu port media */
	int		 cpuport;	/* which PHY is connected to the CPU */
	int		 phymask;	/* PHYs we manage */
	int		 numports;	/* number of ports */
	int		 ifpport[MII_NPHY];
	int		*portphy;
	char		*ifname[B53_NUM_PHYS];
	device_t	 miibus[B53_NUM_PHYS];
	struct ifnet	*ifp[B53_NUM_PHYS];
	struct callout	 callout_tick;

	struct b53_functions	hal;
	struct etherswitch_info	info;
};

#define B53_LOCK(_sc)			mtx_lock(&(_sc)->sc_mtx)
#define B53_UNLOCK(_sc)			mtx_unlock(&(_sc)->sc_mtx)
#define B53_LOCK_ASSERT(_sc, _what)	mtx_assert(&(_sc)->sc_mtx, (_what))
#define B53_TRYLOCK(_sc)		mtx_trylock(&(_sc)->sc_mtx)

/* Read/write access to SWITCH registers via PSEUDO PHY */
uint32_t	b53chip_read4(struct b53_softc *sc, uint32_t reg);
int		b53chip_write4(struct b53_softc *sc, uint32_t reg, uint32_t val);
int		b53chip_op(struct b53_softc *sc, uint32_t reg, uint32_t *res,
		    int is_write);

/* Etherswitch interface */
void		b53switch_lock(device_t dev);
void		b53switch_unlock(device_t dev);
int		b53switch_getvgroup(device_t dev, etherswitch_vlangroup_t *vg);
int		b53switch_setvgroup(device_t dev, etherswitch_vlangroup_t *vg);
int		b53switch_getport(device_t dev, etherswitch_port_t *p);
int		b53switch_setport(device_t dev, etherswitch_port_t *p);

#define B53_RD(_reg, _val, _sc)						\
	do { 								\
		int	b53_err; 					\
		b53_err = b53chip_op(_sc, _reg, &_val, 0);		\
		if (b53_err) {						\
			device_printf(_sc->sc_dev, "can't read"		\
			    " " #_reg ", err: %d\n", b53_err);		\
			return (b53_err);				\
		}							\
	} while (0);

#define B53_WR(_reg, _val, _sc)						\
	do { 								\
		int	b53_err; 					\
		b53_err = b53chip_op(_sc, _reg, &_val, 1);		\
		if (b53_err) {						\
			device_printf(_sc->sc_dev, "can't write"	\
			    " " #_reg ", err: %d\n", b53_err);		\
			return (b53_err);				\
		}							\
	} while (0);

/* Chip operations */
int		b53chip_reset(device_t dev);

/* Common chip actions: */
/*	- enable/disable forwarding */
int		b53chip_enable_fw(device_t dev, uint32_t forward);

#endif /* _B53_VAR_H_ */
