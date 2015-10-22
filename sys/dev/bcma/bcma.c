/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/bus.h>

#include "bcmavar.h"

MALLOC_DEFINE(M_BCMA, "bcma", "BCMA bus data structures");

/**
 * Allocate and initialize new device info descriptor.
 * 
 * @param designer Core designer.
 * @param partnum Core part number.
 * @param revision Hardware revision.
 */
struct bcma_devinfo *
bcma_alloc_dinfo(uint8_t designer, uint8_t partnum, uint8_t revision)
{
	struct bcma_devinfo *dinfo;
	
	dinfo = malloc(sizeof(struct bcma_devinfo), M_BCMA, M_WAITOK);
	if (dinfo == NULL)
		return NULL;
	
	dinfo->cfg.designer = designer;
	dinfo->cfg.partnum = partnum;
	dinfo->cfg.revision = revision;
	
	STAILQ_INIT(&dinfo->cfg.mports);
	STAILQ_INIT(&dinfo->cfg.sports);
	STAILQ_INIT(&dinfo->cfg.wports);

	return dinfo;
}

/**
 * Deallocate the given device info structure and any associated resources.
 * 
 * @param dinfo Device info to be deallocated.
 */
void
bcma_free_dinfo(struct bcma_devinfo *dinfo)
{
	struct bcma_mport *mport, *mnext;
	struct bcma_sport *sport, *snext;

	STAILQ_FOREACH_SAFE(mport, &dinfo->cfg.mports, mp_link, mnext) {
		free(mport, M_BCMA);
	}
	
	STAILQ_FOREACH_SAFE(sport, &dinfo->cfg.sports, sp_link, snext) {
		bcma_free_sport(sport);
	}
	
	STAILQ_FOREACH_SAFE(sport, &dinfo->cfg.wports, sp_link, snext) {
		bcma_free_sport(sport);
	}

	free(dinfo, M_BCMA);
}


/**
 * Allocate and initialize new slave port descriptor.
 * 
 * @param port_num Per-core port number.
 * @param port_type Port type.
 */
struct bcma_sport *
bcma_alloc_sport(uint8_t port_num, bcma_sport_type port_type)
{
	struct bcma_sport *sport;
	
	sport = malloc(sizeof(struct bcma_sport), M_BCMA, M_WAITOK);
	if (sport == NULL)
		return NULL;
	
	sport->sp_num = port_num;
	sport->sp_type = port_type;
	STAILQ_INIT(&sport->sp_maps);

	return sport;
}

/**
 * Deallocate all resources associated with the given port descriptor.
 * 
 * @param sport Port descriptor to be deallocated.
 */
void
bcma_free_sport(struct bcma_sport *sport) {
	struct bcma_map *map, *mapnext;

	STAILQ_FOREACH_SAFE(map, &sport->sp_maps, m_link, mapnext) {
		free(map, M_BCMA);
	}

	free(sport, M_BCMA);
}