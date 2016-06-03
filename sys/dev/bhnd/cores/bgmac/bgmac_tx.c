/*
 * bgmac_tx.c
 *
 *  Created on: Jun 3, 2016
 *      Author: mizhka
 */

/*
 * TX flow
 */

#define	BHND_LOGGING	BHND_TRACE_LEVEL

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <sys/kdb.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#include <net/bpf.h>

#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd_debug.h>

#include "bgmac.h"
#include "bgmacvar.h"
#include "bgmacreg.h"

#include "bcm_dma.h"

/* ***************************************************************
 * 	Internal prototypes
 * ***************************************************************
 */

static int	bgmac_encap(struct bgmac_softc *sc, struct mbuf **m_head);
static void	bgmac_start_locked(struct ifnet *ifp);

void
bgmac_start(struct ifnet *ifp)
{
	struct bgmac_softc *sc;

	sc = if_getsoftc(ifp);
//	BGMAC_LOCK(sc);
	bgmac_start_locked(ifp);
/*	TODO: add locking */
//	BGMAC_UNLOCK(sc);
}

static void
bgmac_start_locked(struct ifnet *ifp)
{
	struct bgmac_softc	*sc;
	struct mbuf		*m_head;
	int			 count;
//	uint32_t prodidx;

	sc = if_getsoftc(ifp);
	/* TODO: add locking */
	//BGMAC_LOCK_ASSERT(sc);

	if ((if_getdrvflags(ifp) & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING)
		return;
//
////	prodidx = sc->bge_tx_prodidx;
//
	for (count = 0; !if_sendq_empty(ifp);) {
		BHND_DEBUG_DEV(sc->dev, "TX queue has new data");

		/* TODO: check if outgoing HW queue is full */

		m_head = if_dequeue(ifp);
		if (m_head == NULL)
			break;

////		if (sc->bge_txcnt > BGE_TX_RING_CNT - 16) {
////			if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, 0);
////			break;
////		}

		/*
		 * Pack the data into the transmit ring. If we
		 * don't have room, set the OACTIVE flag and wait
		 * for the NIC to drain the ring.
		 */
		if (bgmac_encap(sc, &m_head)) {
			if (m_head == NULL)
				break;
			if_sendq_prepend(ifp, m_head);
			if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, 0);
			break;
		}
		++count;
		BHND_DEBUG_DEV(sc->dev,"TX: pkt sent");

		/*
		 * If there's a BPF listener, bounce a copy of this frame
		 * to him.
		 */
		if_bpfmtap(ifp, m_head);
	}
//
//	if (count > 0) {
//		bus_dmamap_sync(sc->bge_cdata.bge_tx_ring_tag,
//		    sc->bge_cdata.bge_tx_ring_map, BUS_DMASYNC_PREWRITE);
//		/* Transmit. */
//		bge_writembx(sc, BGE_MBX_TX_HOST_PROD0_LO, prodidx);
//		/* 5700 b2 errata */
//		if (sc->bge_chiprev == BGE_CHIPREV_5700_BX)
//			bge_writembx(sc, BGE_MBX_TX_HOST_PROD0_LO, prodidx);
//
//		sc->bge_tx_prodidx = prodidx;
//
//		/*
//		 * Set a timeout in case the chip goes out to lunch.
//		 */
//		sc->bge_timer = BGE_TX_TIMEOUT;
//	}
}

static int
bgmac_encap(struct bgmac_softc *sc, struct mbuf **m_head)
{
	struct bcm_dma		*dma;
	int			 err;

	dma = sc->dma;
	err = bcm_dma_tx_start(dma, *m_head);
	if (err) {
		BHND_ERROR_DEV(sc->dev, "bcm_dma_tx_start returns error: %d",
		    err);
		return (err);
	}

	return (-1);
}

