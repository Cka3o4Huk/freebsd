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
#include <sys/errno.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/queue.h>

#include <machine/resource.h>
#include <dev/bhnd/bhndvar.h>
#include <dev/bhnd/bhnd_debug.h>

#include "bus_if.h"
#include "chipc.h"
#include "chipcvar.h"
#include "chipcreg.h"
#include "chipcbus.h"

static void	chipc_print_capabilities(struct chipc_capabilities* capabilities);

int
chipc_init_bus(device_t dev)
{
	int			 err;
	int			 ret;
	struct chipcbus_ivar	*ivar;
	struct chipcbus_reg 	*reg;
	struct chipc_softc	*sc;
	bhnd_addr_t		 rg_start;
	bhnd_size_t		 rg_size;
	device_t 		 bus;

	bus = device_add_child(dev,"bhnd_chipcbus",-1);
	if (bus == NULL) {
		BHND_ERROR_DEV(dev, "error occurred during adding "
				"ChipCommon bus");
		return (ENXIO);
	}

	ivar = malloc(sizeof(struct chipcbus_ivar), M_BHND, M_NOWAIT);
	if (ivar == NULL) {
		BHND_ERROR_DEV(dev, "can't allocate memory for chipcbus_ivar");
		return (ENOMEM);
	}

	device_set_ivars(bus, ivar);

	sc = device_get_softc(dev);
	sc->bus = bus;

	SLIST_INIT(&ivar->mems);
	SLIST_INIT(&ivar->irqs);

	/* Iterate over device ports & regions and fill instance variable */
	for(int i = 0; i < bhnd_get_port_count(dev, BHND_PORT_DEVICE); i++){
		int rgcnt = bhnd_get_region_count(dev, BHND_PORT_DEVICE, i);
		BHND_DEBUG_DEV(dev, "[%d] region count = %d", i, rgcnt);
		for (int j = 0; j < rgcnt; j++){
			ret = bhnd_get_region_addr(dev, BHND_PORT_DEVICE, i, j,
					&rg_start, &rg_size);
			BHND_DEBUG_DEV(dev, "[%d.%d] region addr = 0x%jx (%d)",
					i, j, rg_start, ret);
			if (ret == 0) {
				reg = malloc(sizeof(struct chipcbus_reg),
						M_BHND, M_NOWAIT);
				if (reg == NULL) {
					return (ENOMEM);
				}
				reg->start = rg_start;
				reg->end   = rg_start + rg_size - 1;
				reg->port  = i;
				reg->reg   = j;
				SLIST_INSERT_HEAD(&ivar->mems, reg, entries);
			}
		}
	}

	err = device_probe_and_attach(bus);
	if (err)
		BHND_ERROR_DEV(bus, "ChipCommon bus probing and attaching "
				" failed with error: %d", err);

	return (err);
}

int
chipc_init_pflash(device_t dev, uint32_t flash_config)
{
	//TODO: pass parameters to CFI
	int		width;
	int		enabled;
	int		byteswap;
	int		err;
	device_t	flashdev;

	width = (flash_config & CHIPC_CF_DS) ? 2 : 1;
	enabled = (flash_config & CHIPC_CF_EN);
	byteswap = (flash_config & CHIPC_CF_BS);

	BHND_DEBUG_DEV(dev, "trying attach flash "
			"width=%d enabled=%d swapbytes=%d",
			width, enabled, byteswap);
	flashdev = device_add_child(dev, "cfi", -1);
	if (flashdev == NULL) {
		BHND_ERROR_DEV(dev, "can't add Parallel Flash to "
				"ChipCommon bus");
		return (ENXIO);
	}

	err = device_probe_and_attach(flashdev);
	if (err)
		BHND_ERROR_DEV(dev, "failed attach parallel flash: %d", err);

	return (err);
}

int
chipc_init_sflash(device_t dev, char* flash_name)
{
	device_t	 chipc_spi;
	device_t	*children;
	device_t 	 flash;
	int		 cnt_children;
	int		 err;

	chipc_spi = device_add_child(dev, "spi", -1);
	if (chipc_spi == NULL) {
		BHND_ERROR_DEV(dev, "can't add chipc_spi to ChipCommon");
		return (ENXIO);
	}

	err = device_probe_and_attach(chipc_spi);
	if (err) {
		BHND_ERROR_DEV(dev, "failed attach chipc_spi: %d", err);
		return (err);
	}

	err = device_get_children(chipc_spi, &children, &cnt_children);
	if (err) {
		BHND_ERROR_DEV(chipc_spi, "can't get list of children: %d",
				err);
		return (err);
	}

	if (cnt_children == 0) {
		BHND_ERROR_DEV(chipc_spi, "no found children");
		return (ENXIO);
	}

	flash = BUS_ADD_CHILD(children[0], 0, flash_name, -1);
	if (children)
		free(children, M_TEMP);

	if (flash == NULL) {
		BHND_ERROR_DEV(chipc_spi, "can't add %s to spibus", flash_name);
		return (ENXIO);
	}

	err = device_probe_and_attach(flash);
	if (err)
		BHND_ERROR_DEV(dev, "failed attach flash %s: %d",
				flash_name, err);

	return (err);
}

int
chipc_init_uarts(struct chipc_softc* sc, uint8_t num_uarts)
{
	device_t 	child;
	int 		err;

	if (num_uarts == 0)
		return (0);

	child = device_add_child(sc->bus, "uart", 0);
	if (child == NULL) {
		BHND_ERROR_DEV(sc->bus, "can'd add UART to bus");
		return (ENXIO);
	}

	err = device_probe_and_attach(child);
	if (err)
		BHND_ERROR_DEV(child, "error occurred on probe_and_attach: %d",
				err);

	return (err);
}

static void
chipc_print_capabilities(struct chipc_capabilities* capabilities)
{

	BHND_DEBUG("UARTs: 0x%01x", capabilities->num_uarts);
	BHND_DEBUG("BigEngian: 0x%01x", capabilities->is_bigend);
	BHND_DEBUG("UART-GPIO: 0x%01x", capabilities->uart_gpio);
	BHND_DEBUG("UART Clock: 0x%01x", capabilities->uart_clock);
	BHND_DEBUG("Flash type: 0x%x", capabilities->flash_type);

	BHND_DEBUG("External buses: 0x%x",  capabilities->external_buses);
	BHND_DEBUG("Power control: 0x%01x", capabilities->power_control);
	BHND_DEBUG("JTAG master: 0x%01x", capabilities->jtag_master);

	BHND_DEBUG("PLL Type: 0x%x", capabilities->pll_type);
	BHND_DEBUG("OTP size: 0x%01x", capabilities->otp_size);
	BHND_DEBUG("Is 64bit? 0x%01x", capabilities->is_64bit);
	BHND_DEBUG("Boot ROM: 0x%01x", capabilities->boot_rom);
	BHND_DEBUG("PMU: 0x%01x", capabilities->pmu);
	BHND_DEBUG("ECI: 0x%01x", capabilities->eci);
	BHND_DEBUG("SPROM: 0x%01x", capabilities->sprom);
}

void
chipc_parse_capabilities(struct chipc_capabilities* capabilities,
		u_int32_t caps)
{

	capabilities->num_uarts = GET_BITS(caps, CHIPC_CAP_UARTS);
	capabilities->is_bigend = GET_BITS(caps, CHIPC_CAP_MIPSEB);
	capabilities->uart_gpio = GET_BITS(caps, CHIPC_CAP_UARTGPIO);
	capabilities->uart_clock= GET_BITS(caps, CHIPC_CAP_UCLKSEL);
	capabilities->flash_type= GET_BITS(caps, CHIPC_CAP_FLASH);

	capabilities->external_buses = GET_BITS(caps, CHIPC_CAP_EXTBUS);
	capabilities->power_control = GET_BITS(caps, CHIPC_CAP_PWR_CTL);
	capabilities->jtag_master = GET_BITS(caps, CHIPC_CAP_JTAGP);

	capabilities->pll_type = GET_BITS(caps, CHIPC_CAP_PLL);
	capabilities->otp_size = GET_BITS(caps, CHIPC_CAP_OTP_SIZE);
	capabilities->is_64bit = GET_BITS(caps, CHIPC_CAP_BKPLN64);
	capabilities->boot_rom = GET_BITS(caps, CHIPC_CAP_ROM);
	capabilities->pmu = GET_BITS(caps, CHIPC_CAP_PMU);
	capabilities->eci = GET_BITS(caps, CHIPC_CAP_ECI);
	capabilities->sprom = GET_BITS(caps, CHIPC_CAP_SPROM);

	if(bootverbose)
		chipc_print_capabilities(capabilities);
}
