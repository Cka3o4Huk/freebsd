/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
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

/*
 * Slicer is required to split firmware images into pieces.
 * The first supported FW is TRX-based used by Asus routers
 * TODO: add NetGear FW (CHK)
 */

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

#include "chipc_slicer.h"

#include <dev/cfi/cfi_var.h>
#include "chipc_spi.h"

struct chipc_slicer_info {
	uint32_t	fw_offset;
	uint32_t	fw_size;
	uint32_t	fw_end;
	uint32_t	fw_magic;
	uint32_t	total_size;
};

static int	chipc_slicer_walk(device_t dev, struct resource *res,
		    struct flash_slice *slices, int *nslices);
static int	chipc_slicer_scan(device_t dev, struct resource *res,
		    uint32_t *fwmagics, uint32_t *auxmagics,
		    struct chipc_slicer_info *out);

void
chipc_register_slicer(chipc_flash flash_type)
{
	switch (flash_type) {
	case CHIPC_SFLASH_AT:
	case CHIPC_SFLASH_ST:
		flash_register_slicer(chipc_slicer_spi, FLASH_SLICES_TYPE_SPI,
		   TRUE);
		break;
	case CHIPC_PFLASH_CFI:
		flash_register_slicer(chipc_slicer_cfi, FLASH_SLICES_TYPE_CFI,
		   TRUE);
		break;
	default:
		/* Unsupported */
		break;
	}
}

int
chipc_slicer_cfi(device_t dev, const char *provider __unused,
    struct flash_slice *slices, int *nslices)
{
	struct cfi_softc	*sc;
	device_t		 parent;

	/* must be CFI flash */
	if (device_get_devclass(dev) != devclass_find("cfi"))
		return (ENXIO);

	/* must be attached to chipc */
	if ((parent = device_get_parent(dev)) == NULL) {
		BHND_ERROR_DEV(dev, "no found ChipCommon device");
		return (ENXIO);
	}

	if (device_get_devclass(parent) != devclass_find("bhnd_chipc")) {
		BHND_ERROR_DEV(dev, "no found ChipCommon device");
		return (ENXIO);
	}

	sc = device_get_softc(dev);
	return (chipc_slicer_walk(dev, sc->sc_res, slices, nslices));
}

int
chipc_slicer_spi(device_t dev, const char *provider __unused,
    struct flash_slice *slices, int *nslices)
{
	struct chipc_spi_softc	*sc;
	device_t		 chipc, spi, spibus;
	char			*err_msg = "OK";

	BHND_DEBUG_DEV(dev, "initting SPI slicer: %s", device_get_name(dev));

	/* must be SPI-attached flash */
	spibus = device_get_parent(dev);
	if (spibus == NULL) {
		err_msg = "no found ChipCommon SPI BUS device";
		goto error;
	}

	spi = device_get_parent(spibus);
	if (spi == NULL) {
		err_msg = "no found ChipCommon SPI device";
		goto error;
	}

	chipc = device_get_parent(spi);
	if (device_get_devclass(chipc) != devclass_find("bhnd_chipc")) {
		err_msg = "no found ChipCommon device";
		goto error;
	}

	sc = device_get_softc(spi);
	return (chipc_slicer_walk(dev, sc->sc_flash_res, slices, nslices));
error:
	BHND_ERROR_DEV(dev, "%s", err_msg);
	return (ENXIO);
}

static int
chipc_slicer_scan(device_t dev, struct resource *res, uint32_t *fwmagics,
    uint32_t *auxmagics, struct chipc_slicer_info *out)
{
	uint32_t			*curmagic;
	uint32_t			 offset;
	uint32_t			 val;

	out->fw_offset = UINT32_MAX;
	out->total_size = out->fw_size = rman_get_size(res);

	BHND_TRACE_DEV(dev, "slicer: scanning memory [%x bytes] for headers...",
	    out->total_size);
	for (offset = 0; offset < out->total_size; offset += CHIPC_SLICER_STEP) {
		val = bus_read_4(res, offset);

		/* Check end of firmware image - new magic number */
		if ((out->fw_offset < offset) && (out->fw_size > offset))
			for(curmagic = auxmagics; *curmagic != 0; curmagic++) {
				if (val != *curmagic)
					continue;

				out->fw_size = offset - out->fw_offset;
				break;
			}

		for (curmagic = fwmagics; *curmagic != 0; curmagic++) {
			if (val != *curmagic)
				continue;

			if (offset < out->fw_offset) {
				out->fw_offset = offset;
				out->fw_magic = *curmagic;
				continue;
			}
			/* Second shadow of firmware magic is found - STOP */
			out->total_size = offset - out->fw_offset;

			/*
			 * If firmware image is last partition on flash, we
			 * need to adjust fw_size
			 */
			out->fw_size = MIN(out->fw_size,
					out->total_size - out->fw_offset);
			out->fw_end = out->fw_size + out->fw_offset;
			return (0);
		}
	}

	return (-1);
}

uint32_t	fw_magics[] = { TRX_MAGIC, 0 };
uint32_t	aux_magics[] = { CFE_MAGIC, NVRAM_MAGIC, 0 };

/*
 * Main processing part
 */
static int
chipc_slicer_walk(device_t dev, struct resource *res,
    struct flash_slice *slices, int *nslices)
{
	struct chipc_slicer_info	 result;
	struct flash_slice		*tmp;
	uint32_t	 		 fs_ofs;
	uint32_t			 fw_len;
	int				 err, cnt;

	*nslices = 0;
	cnt = 0;
	tmp = slices;

	err = chipc_slicer_scan(dev, res, fw_magics, aux_magics, &result);
	if (err != 0) {
		BHND_ERROR_DEV(dev, "slicer: can't identify flash size");
		return (0);
	}

	BHND_INFO_DEV(dev, "Firmware image [0x%x]: 0x%x - 0x%x",
			result.fw_magic,
			result.fw_offset,
			result.fw_end);

	switch (result.fw_magic) {
	case TRX_MAGIC:
		/* read last offset of TRX header */
		fw_len = bus_read_4(res, result.fw_offset + 4);
		fs_ofs = bus_read_4(res, result.fw_offset + 24);
		BHND_TRACE_DEV(dev, "TRX filesystem: 0x%x-0x%x", fs_ofs,
		   fw_len + result.fw_offset);

		/*
		 * GEOM IO will panic if offset is not aligned
		 * on sector size, i.e. 512 bytes
		 */
		if ((fs_ofs & 0x1FF) != 0) {
			BHND_WARN_DEV(dev, "WARNING! filesystem offset should be"
			    " aligned on sector size (%d bytes)\n"
			    " ignoring TRX firmware image", 0x200);
			break;
		}

		tmp->base = result.fw_offset + fs_ofs;
		tmp->size = fw_len - fs_ofs;
		tmp->label = "rootfs";

		BHND_TRACE_DEV(dev, "TRX filesystem rootfs: 0x%x-0x%x", (unsigned)tmp->base,
		   (unsigned)(tmp->base + tmp->size));
		cnt++;
		tmp++;

		if (fw_len + CHIPC_SLICER_CFGSIZE > result.fw_size)
		{
			/* no cfg slice, break */
			break;
		}
		/* configuration slice */
		tmp->size = CHIPC_SLICER_CFGSIZE;
		tmp->base = ((result.fw_end - CHIPC_SLICER_CFGSIZE) &
		    ~(CHIPC_SLICER_CFGSIZE - 1));
		tmp->label = "cfg";
		BHND_TRACE_DEV(dev, "TRX filesystem cfg: 0x%x-0x%x", (unsigned)tmp->base,
		   (unsigned)(tmp->base + tmp->size));
		cnt++;
		break;
	default:
		break;
	}

	BHND_TRACE("slicer: found %d partitions", cnt);
	*nslices = cnt;
	return (0);
}
