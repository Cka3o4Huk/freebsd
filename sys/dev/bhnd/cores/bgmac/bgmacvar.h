/*-
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

#pragma once

#include <sys/param.h>
#include <sys/malloc.h>
#include <sys/kernel.h>

#define BGMAC_TIMEOUT 1000
#define BGMAC_MAX_RSPEC	3

MALLOC_DECLARE(M_BHND_BGMAC);
MALLOC_DEFINE(M_BHND_BGMAC, "bgmac", "Structures allocated by bgmac driver");

struct bgmac_softc {
	device_t 		 dev;
	device_t		 miibus;
	device_t		 mdio;
	struct resource		*mem;
	struct resource		*irq;
	void			*intrhand;
	struct ifnet		*ifp;
	u_char                   addr[6];
	struct bcm_dma		*dma;
//	bus_dma_tag_t		 parent_tag, ring_tag;
//	bus_dmamap_t		 ring_map;
//	bus_addr_t		 rxdesc_ring_busaddr;
//	void			*buf;
//	void			*rxdesc_ring;
};

typedef enum{
	PHY_READ,
	PHY_WRITE
}  phymode ;
