/*
 * chipc_slicer.c
 *
 *  Created on: Apr 25, 2016
 *      Author: mizhka
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

#include <machine/bus.h>

#include <dev/bhnd/bhnd_debug.h>
#include <dev/cfi/cfi_var.h>

#include "chipc_spi.h"
#include "flash_if.h"

#include <sys/slicer.h>

#define TRX_MAGIC 	0x30524448
#define CFE_MAGIC 	0x43464531
#define NVRAM_MAGIC	0x48534C46


int chipc_slicer_walk(struct resource* res, int flash_size, struct flash_slice *slices, int *nslices);
int chipc_slicer_flash(device_t dev, struct flash_slice *slices, int *nslices);

/*
 * Slicer is required to split firmware images into pieces.
 * The first supported FW is TRX-based used by Asus routers
 * TODO: add NetGear FW (CHK)
 */
int chipc_slicer_flash(device_t dev, struct flash_slice *slices, int *nslices){
	struct resource* res;
	int flash_size;
	const char* devclazz = device_get_name(dev);

	BHND_INFO_DEV(dev, ("start flash slicer for class %s",
			device_get_name(dev)));

	if (strcmp(devclazz, "mx25l") == 0) { //SPI
		//flash(mx25l) <- spibus <- chipc_spi
		device_t spibus = device_get_parent(dev);
		device_t chipc_spi = device_get_parent(spibus);
		struct chipc_spi_softc *sc = device_get_softc(chipc_spi);

		flash_size = FLASH_GET_SIZE(dev);
		res = sc->sc_res;

	}else if(strcmp(devclazz, "cfi") == 0){ //CFI
		struct cfi_softc *sc = device_get_softc(dev);

		flash_size = sc->sc_size;
		res = sc->sc_res;
	}else {
		//Unsupported case
		BHND_ERROR_DEV(dev, ("unsupported flash device class: %s", device_get_name(dev)))
		return (0);
	}

	/*
	 * Main processing part
	 */
	return chipc_slicer_walk(res,flash_size,slices,nslices);
}

int chipc_slicer_walk(struct resource* res, int flash_size, struct flash_slice *slices, int *nslices){
	bus_space_tag_t mem_tag		= rman_get_bustag(res);
	bus_space_handle_t mem_hndl 	= rman_get_bushandle(res);

	*nslices = 0;
	BHND_TRACE(("slicer: scanning memory for headers...\n"));

	/* Find FW header in flash memory with step = 0x1000 (or block size (128Kb)? */
	for(uint32_t ofs = 0; ofs < flash_size; ofs+= 0x1000){
		uint32_t val = bus_space_read_4(mem_tag, mem_hndl, ofs);
		switch(val){
		case TRX_MAGIC:
			BHND_DEBUG(("TRX found: %x\n", ofs));
			//read last offset of TRX header
			uint32_t fs_ofs = bus_space_read_4(mem_tag, mem_hndl, ofs + 24);
			BHND_DEBUG(("FS offset: %x\n", fs_ofs));

			/*
			 * GEOM IO will panic if offset is not aligned on sector size, i.e. 512 bytes
			 */
			if(fs_ofs % 0x200 != 0){
				BHND_ERROR(("WARNING! filesystem offset should be aligned on sector size (%d bytes)\n", 0x200));
				break;
			}

			slices[*nslices].base = ofs + fs_ofs;
			//XXX: fully sized? any other partition?
			uint32_t fw_len = bus_space_read_4(mem_tag, mem_hndl, ofs + 4);
			slices[*nslices].size = fw_len - fs_ofs;
			slices[*nslices].label = "rootfs";
			*nslices += 1;
			break;
		case CFE_MAGIC:
			BHND_DEBUG(( "CFE found: %x\n", ofs ));
			break;
		case NVRAM_MAGIC:
			BHND_DEBUG(( "NVRAM found: %x\n", ofs ));
			break;
		default:
			break;
		}
	}

	BHND_TRACE(("slicer: done\n"));
	return (0);
}

