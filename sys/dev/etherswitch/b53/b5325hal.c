/*-
 * Copyright (c) 2011 Aleksandr Rybalko.
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * Ported version of bcm5325_switch from ZRouter.org
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/errno.h>
#include <sys/bus.h>

#include "b53_var.h"
#include "b53_reg.h"
#include "b53_hal.h"

static int	b5325hal_get_port_pvid(struct b53_softc *sc, int port, int *pvid);
static int	b5325hal_set_port_pvid(struct b53_softc *sc, int port, int pvid);
static int	b5325hal_reset(struct b53_softc *sc);

struct b53_functions b5325_f = {
	.reset = b5325hal_reset,
	.vlan_get_pvid = b5325hal_get_port_pvid,
	.vlan_set_pvid = b5325hal_set_port_pvid
	/* TODO: add VLAN getter/setter */
};

struct b53_hal	b5325_hal = {
	.parent = NULL,
	.own = &b5325_f
};

static int
b5325hal_reset(struct b53_softc *sc)
{
	int		err;
	uint32_t	reg;

	/* MII port state override (page 0 register 14) */
	err = b53chip_op(sc, PORTMII_STATUS_OVERRIDE, &reg, 0);

	if (err) {
		device_printf(sc->sc_dev, "Unable to set RvMII mode\n");
		return (ENXIO);
	}

	/* Bit 4 enables reverse MII mode */
	if (!(reg & PORTMII_STATUS_REVERSE_MII))
	{
		/* Enable RvMII */
		reg |= PORTMII_STATUS_REVERSE_MII;
		b53chip_write4(sc, PORTMII_STATUS_OVERRIDE, reg);
		/* Read back */
		err = b53chip_op(sc, PORTMII_STATUS_OVERRIDE, &reg, 0);
		if (err || !(reg & PORTMII_STATUS_REVERSE_MII))
		{
			device_printf(sc->sc_dev, "Unable to set RvMII mode\n");
			return (ENXIO);
		}
	}

	return (0);
}

static int
b5325hal_get_port_pvid(struct b53_softc *sc, int port, int *pvid)
{
	int	local_pvid;

	local_pvid = b53chip_read4(sc, VLAN_DEFAULT_PORT_TAG(port));
	if (local_pvid < 0)
		return (EBUSY);

	*pvid = local_pvid & 0xfff; /* TODO: define macro for mask */
	return (0);
}

int
b5325hal_set_port_pvid(struct b53_softc *sc, int port, int pvid)
{

	if (port > (sc->info.es_nports - 1))
		return (EINVAL);

	if (pvid > 0xfff)
		return (EINVAL);

	return (b53chip_write4(sc, VLAN_DEFAULT_PORT_TAG(port), pvid));
}
