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
 *
 * Broadcom Gigabit MAC
 *
 * On Asus RT-N16 GMAC core attaches BCM53115 chip to SoC via GMII. So this driver is
 * MII-based. Information about registers are taken from:
 *      http://bcm-v4.sipsolutions.net/mac-gbit/Registers
 *
 */

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/socket.h>

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

#include "miibus_if.h"

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miidevs.h"
#include <dev/mii/brgphyreg.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhnd_ids.h>

#include "bgmacvar.h"
#include "bgmacreg.h"

static const struct resource_spec bgmac_rspec[BGMAC_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

static struct bhnd_core_id bgmac_ids[] = {
		{BHND_MFGID_BCM, BHND_COREID_GMAC},
		{-1,-1}
};

/**
 * Prototypes
 */

static int bgmac_probe(device_t dev);
static int bgmac_attach(device_t dev);

static int bgmac_readreg(device_t dev,int phy,int reg);
static int bgmac_writereg(device_t dev,int phy,int reg, int val);

int bgmac_phyreg_poll(device_t dev,uint32_t reg, uint32_t mask);
int bgmac_phyreg_op(device_t dev, phymode op, int phy,int reg,int* val);

/**
 * Implementation
 */

static int bgmac_probe(device_t dev){
	uint32_t mfg = bhnd_get_vendor(dev);
	uint32_t devid = bhnd_get_device(dev);

	for( struct bhnd_core_id* testid = bgmac_ids; testid->mfg != -1; testid++ ){
		if(mfg == testid->mfg && devid == testid->devid){
			device_set_desc(dev, "BHND GMAC");
			return (BUS_PROBE_DEFAULT);
		}
	}
	return (ENXIO);
}

static int bgmac_attach(device_t dev){

	struct bgmac_softc* sc = device_get_softc(dev);
	sc->dev = dev;

	/* Allocate bus resources */
	memcpy(sc->rspec, bgmac_rspec, sizeof(sc->rspec));
	int error = bhnd_alloc_resources(dev, sc->rspec, sc->res);
	if(error)
		return (error);

	struct resource* res = sc->res[0]->res;

	if(!res){
		return (ENXIO);
	}

	sc->hdl = rman_get_bushandle(res);
	sc->tag = rman_get_bustag(res);

	return 0;
}

int bgmac_phyreg_poll(device_t dev,uint32_t reg, uint32_t mask){
	struct bgmac_softc* sc = device_get_softc(dev);
	/* Poll for the register to complete. */
	for (int i = 0; i < BGMAC_TIMEOUT; i++) {
		DELAY(10);
		uint32_t val = bus_space_read_4(sc->tag, sc->hdl, reg);
		if ((val & mask) == 0) {
			DELAY(5);
			return 0;
		}
	}
	return -1;
}

int bgmac_phyreg_op(device_t dev, phymode op, int phy,int reg,int* val){
	struct bgmac_softc* sc = device_get_softc(dev);

	//Set address on PHY control register
	uint32_t tmp = bus_space_read_4(sc->tag, sc->hdl, BGMAC_REG_PHY_CONTROL);
	tmp = (tmp & (~BGMAC_REG_PHY_ACCESS_ADDR)) | phy;
	bus_space_write_4(sc->tag, sc->hdl, BGMAC_REG_PHY_CONTROL, tmp);

	//Send header (first 16 bytes) over MII
	tmp = BGMAC_REG_PHY_ACCESS_START;
	if(op == PHY_WRITE){
		tmp |= BGMAC_REG_PHY_ACCESS_WRITE;
		tmp |= (*val & BGMAC_REG_PHY_ACCESS_DATA);
	}
	tmp |= phy << BGMAC_REG_PHY_ACCESS_ADDR_SHIFT;
	tmp |= reg << BGMAC_REG_PHY_ACCESS_REG_SHIFT;

	bus_space_write_4(sc->tag, sc->hdl, BGMAC_REG_PHY_ACCESS, tmp);

	//Wait while operation is finished
	if(bgmac_phyreg_poll(dev, BGMAC_REG_PHY_ACCESS, BGMAC_REG_PHY_ACCESS_START)){
		return -1;
	}

	if(op == PHY_READ){
		//Read rest of 16 bytes back
		tmp = bus_space_read_4(sc->tag, sc->hdl, BGMAC_REG_PHY_ACCESS);
		tmp &= BGMAC_REG_PHY_ACCESS_DATA;
		*val = tmp;
	}
	return 0;
}

static int bgmac_readreg(device_t dev,int phy,int reg){
	int tmp;
	if(bgmac_phyreg_op(dev, PHY_READ,phy,reg,&tmp)){
		return -1;
	}
	return tmp;
}

static int bgmac_writereg(device_t dev,int phy,int reg, int val){
	int tmp = val;
	if(bgmac_phyreg_op(dev, PHY_WRITE,phy,reg,&tmp)){
		return -1;
	}
	return 0;
}

/**
 * Driver metadata
 */

static device_method_t bgmac_methods[] = {
		DEVMETHOD(device_probe,	 bgmac_probe),
		DEVMETHOD(device_attach, bgmac_attach),
		/** miibus interface **/
		DEVMETHOD(miibus_readreg, 	bgmac_readreg),
		DEVMETHOD(miibus_writereg, 	bgmac_writereg),

		DEVMETHOD_END
};

devclass_t bhnd_bgmac_devclass;

DEFINE_CLASS_0(bhnd_bgmac, bgmac_driver, bgmac_methods, sizeof(struct bgmac_softc));
DRIVER_MODULE(bhnd_bgmac, bhnd, bgmac_driver, bhnd_bgmac_devclass, 0, 0);
MODULE_VERSION(bhnd_bgmac, 1);
