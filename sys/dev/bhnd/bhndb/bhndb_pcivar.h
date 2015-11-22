/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * 
 * $FreeBSD$
 */

#ifndef _BHND_BHNDB_PCIVAR_H_
#define _BHND_BHNDB_PCIVAR_H_

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <dev/bhnd/bhnd.h>

#include "bhndbvar.h"

DECLARE_CLASS(bhndb_pci_driver);

struct bhndb_pci_regwin_region;

/**
 * bhndb driver instance state. Must be first member of all subclass
 * softc structures.
 */
struct bhndb_pci_softc {
	device_t			 dev;		/**< bridge device */
	const struct bhndb_hwcfg	*hwcfg;		/**< hardware config */

	device_t			 pci_dev;	/**< parent pci device */
	size_t				 res_count;	/**< pci resource count */
	struct resource_spec		*res_spec;	/**< pci resource specs */
	struct resource			**res;		/**< pci resources */
	struct rman			 mem_rman;	/**< bus memory manager */

	struct mtx			 res_mtx;	/**< resource allocator lock. */
	
	struct bhndb_pci_regwin_region	*dw_regions;	/**< dynamic window regions */
	size_t				 dw_count;	/**< number of dynamic window regions. */
	uint32_t			 dw_freelist;	/**< dw_regions free list */
};

/**
 * bhndb child instance state. Must be first member of all subclass
 * devinfo structures.
 */
struct bhndb_devinfo {
        struct resource_list    resources;		/**< bridged resources. */
};

int	bhndb_pci_probe(device_t dev);
int	bhndb_pci_attach(device_t dev);
int	bhndb_pci_detach(device_t dev);
int	bhndb_pci_suspend(device_t dev);
int	bhndb_pci_resume(device_t dev);
int	bhndb_pci_read_ivar(device_t dev, device_t child, int index,
	    uintptr_t *result);
int	bhndb_pci_write_ivar(device_t dev, device_t child, int index,
	    uintptr_t value);

#define	BHNDB_RES_LOCK_INIT(sc) \
	mtx_init(&(sc)->res_mtx, device_get_nameunit((sc)->dev), \
	    "bhndb_pci resource allocator lock", MTX_DEF)
#define	BHNDB_RES_LOCK(sc)		mtx_lock(&(sc)->res_mtx)
#define	BHNDB_RES_UNLOCK(sc)		mtx_unlock(&(sc)->res_mtx)
#define	BHNDB_RES_LOCK_ASSERT(sc)	mtx_assert(&(sc)->res_mtx, MA_OWNED)
#define	BHNDB_RES_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->res_mtx)

/* Mark a dw_region as free */
#define	BHNDB_DWR_RELEASE(sc, idx)	do {			\
	KASSERT((sc)->dw_regions[idx].child_res != NULL &&	\
	    !BHNDB_DWR_IS_FREE((sc), (idx)),			\
	    (("dw_region double free")));			\
								\
	(sc)->dw_freelist |= (1 << (idx));			\
	(sc)->dw_regions[idx].child_res = NULL;			\
} while(0)

/* Mark a dw_region as reserved */
#define	BHNDB_DWR_RESERVE(sc, idx, cr)	do {			\
	KASSERT((sc)->dw_regions[idx].child_res == NULL &&	\
	    BHNDB_DWR_IS_FREE((sc), (idx)),			\
	    (("dw_region is busy")));				\
								\
	(sc)->dw_freelist &= ~(1 << (idx));			\
	(sc)->dw_regions[idx].child_res = cr;			\
} while(0)

/* Return non-zero value if the DWR region is free */
#define	BHNDB_DWR_IS_FREE(sc, idx)	((sc)->dw_freelist & (1 << (idx)))

#endif /* _BHND_BHND_PCIVAR_H_ */