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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/errno.h>
#include <sys/rman.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/slicer.h>

#include <machine/bus.h>

#include <dev/bhnd/bhnd_debug.h>
#include <dev/cfi/cfi_var.h>

#include "chipc_spi.h"
#include "flash_if.h"

#define	TRX_MAGIC 	0x30524448
#define	CFE_MAGIC 	0x43464531
#define	NVRAM_MAGIC	0x48534C46


static int	chipc_slicer_walk(struct resource* res, int flash_size,
		    struct flash_slice *slices, int *nslices);
int		chipc_slicer_flash(device_t dev, struct flash_slice *slices,
		    int *nslices);

/*
 * Slicer is required to split firmware images into pieces.
 * The first supported FW is TRX-based used by Asus routers
 * TODO: add NetGear FW (CHK)
 */
int
chipc_slicer_flash(device_t dev, struct flash_slice *slices, int *nslices)
{
	struct resource	*res;
	int		 flash_size;
	const char	*devclazz;

	devclazz = device_get_name(dev);
	BHND_INFO_DEV(dev, "start flash slicer for class %s", devclazz);
	if (strcmp(devclazz, "mx25l") == 0) { //SPI
		/* flash(mx25l) <- spibus <- chipc_spi */
		device_t		 spibus;
		device_t		 chipc_spi;
		struct chipc_spi_softc	*sc;

		spibus = device_get_parent(dev);
		chipc_spi = device_get_parent(spibus);
		sc = device_get_softc(chipc_spi);

		flash_size = FLASH_GET_SIZE(dev);
		res = sc->sc_res;
	} else if(strcmp(devclazz, "cfi") == 0) {
		/* cfi */
		struct cfi_softc *sc = device_get_softc(dev);
		flash_size = sc->sc_size;
		res = sc->sc_res;
	}else {
		/* Unsupported case */
		BHND_ERROR_DEV(dev, "ChipCommon Flash slicer: "
				"unsupported flash device class: %s", devclazz);
		return (0);
	}

	/*
	 * Main processing part
	 */
	return (chipc_slicer_walk(res,flash_size,slices,nslices));
}

static int
chipc_slicer_walk(struct resource* res, int flash_size,
		struct flash_slice *slices, int *nslices)
{
	uint32_t	fw_len;
	uint32_t	fs_ofs;
	uint32_t	val;

	*nslices = 0;
	BHND_TRACE("slicer: scanning memory for headers...");

	/*
	 * Find FW header in flash memory with step = 0x1000
	 * (or block size (128Kb)?
	 */
	for(uint32_t ofs = 0; ofs < flash_size; ofs+= 0x1000){
		val = bus_read_4(res, ofs);
		switch (val) {
		case TRX_MAGIC:
			BHND_DEBUG("TRX found: %x", ofs);
			/* read last offset of TRX header */
			fs_ofs = bus_read_4(res, ofs + 24);
			BHND_DEBUG("FS offset: %x", fs_ofs);

			/*
			 * GEOM IO will panic if offset is not aligned
			 * on sector size, i.e. 512 bytes
			 */
			if (fs_ofs % 0x200 != 0) {
				BHND_WARN("WARNING! filesystem offset should be"
				    " aligned on sector size (%d bytes)", 0x200);
				BHND_WARN("ignoring firmware image");
				break;
			}

			slices[*nslices].base = ofs + fs_ofs;
			//XXX: fully sized? any other partition?
			fw_len = bus_read_4(res, ofs + 4);
			slices[*nslices].size = fw_len - fs_ofs;
			slices[*nslices].label = "rootfs";
			*nslices += 1;
			break;
		case CFE_MAGIC:
			BHND_DEBUG("CFE found: %x", ofs);
			break;
		case NVRAM_MAGIC:
			BHND_DEBUG("NVRAM found: %x", ofs);
			break;
		default:
			break;
		}
	}

	BHND_TRACE("slicer: done");
	return (0);
}
