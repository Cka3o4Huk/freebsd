/*
 * bcmswitchvar.h
 *
 *  Created on: May 25, 2016
 *      Author: mizhka
 */

#ifndef SYS_DEV_ETHERSWITCH_B53SWITCH_B53SWITCHVAR_H_
#define SYS_DEV_ETHERSWITCH_B53SWITCH_B53SWITCHVAR_H_

#include <sys/malloc.h>

#include <dev/mii/mii.h>

#define	B53SWITCH_NUM_PHYS	5

MALLOC_DECLARE(M_BCMSWITCH);
MALLOC_DEFINE(M_BCMSWITCH, "bcmswitch", "bcmswitch data structures");

struct b53switch_softc {
	struct mtx	sc_mtx;		/* serialize access to softc */
	device_t	sc_dev;
	device_t	sc_parent;
	int		media;		/* cpu port media */
	int		cpuport;	/* which PHY is connected to the CPU */
	int		phymask;	/* PHYs we manage */
	int		numports;	/* number of ports */
	int		ifpport[MII_NPHY];
	int		*portphy;
	char		*ifname[B53SWITCH_NUM_PHYS];
	device_t	miibus[B53SWITCH_NUM_PHYS];
	struct ifnet	*ifp[B53SWITCH_NUM_PHYS];
	struct callout	callout_tick;
	struct etherswitch_info	info;
};


#endif /* SYS_DEV_ETHERSWITCH_B53SWITCH_B53SWITCHVAR_H_ */
