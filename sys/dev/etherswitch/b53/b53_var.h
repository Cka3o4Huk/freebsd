/*
 * bcmswitchvar.h
 *
 *  Created on: May 25, 2016
 *      Author: mizhka
 */

#ifndef SYS_DEV_ETHERSWITCH_B53_B53_VAR_H_
#define SYS_DEV_ETHERSWITCH_B53_B53_VAR_H_

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

#define	B53_NUM_PHYS	6

MALLOC_DECLARE(M_BCMSWITCH);

struct b53_softc;
struct b53_hal;

#define	B53HALSIZE		4

typedef void (*voidfunctype) (void);

struct b53_functions {
	union{
		struct {
			/* VLAN functions */
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
		    int is_wr);
/* Chip operations */
int		b53chip_reset(device_t dev);

#endif /* SYS_DEV_ETHERSWITCH_B53_B53_VAR_H_ */
