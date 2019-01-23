/*-
 * Copyright (c) 2018 Michael Zhilin <mizhka@freebsd.org>
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
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/rman.h>
#include <sys/sbuf.h>

#include "robosw_var.h"
#include "robosw_reg.h"

int
robosw_init_sysctl(device_t dev)
{
	struct robosw_softc	*sc;
	struct sysctl_ctx_list	*ctx;
	struct sysctl_oid	*tree, *stats_node, *port_node;
	int			 i;
	char 			 name[IFNAMSIZ];

	sc = device_get_softc(dev);
	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);

	SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "debug", CTLFLAG_RW, &sc->sc_debug, 0, "control debugging printfs");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "registers",
	    CTLFLAG_RD | CTLTYPE_STRING, sc, 0, robosw_sysctl_dump, "A",
	    "dump of registers");

	if (sc->hal.api.mib_get == NULL)
		return (0);

	stats_node = SYSCTL_ADD_NODE(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "stats", CTLFLAG_RW, 0, "statistics");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(stats_node), OID_AUTO, "cpu_rx",
	    CTLFLAG_RD | CTLTYPE_U16, sc, (sc->cpuport << 4) | ROBOSW_MIB_GOODRXPKTS,
	    robosw_mib, "I", "number of packets received by port");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(stats_node), OID_AUTO, "cpu_tx",
	    CTLFLAG_RD | CTLTYPE_U16, sc, (sc->cpuport << 4)| ROBOSW_MIB_GOODTXPKTS,
	    robosw_mib, "I", "number of packets sent by port");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(stats_node), OID_AUTO, "reset",
	    CTLFLAG_RW | CTLTYPE_U16, sc, 0, robosw_mib_reset, "I",
	    "reset chip statistic");

	for (i = 0; i < sc->info.es_nports - 1; i++) {
		snprintf(name, IFNAMSIZ, "port%d", i);

		port_node = SYSCTL_ADD_NODE(ctx, SYSCTL_CHILDREN(stats_node),
				OID_AUTO, name, CTLFLAG_RW, 0, "port counters");

		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(port_node), OID_AUTO,
		    "rx", CTLFLAG_RD | CTLTYPE_U16, sc,
		    (i << 4) | ROBOSW_MIB_GOODRXPKTS, robosw_mib, "I",
		    "number of packets received by port");

		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(stats_node), OID_AUTO,
		    "tx", CTLFLAG_RD | CTLTYPE_U16, sc,
		    (i << 4) | ROBOSW_MIB_GOODTXPKTS, robosw_mib, "I",
		    "number of packets sent by port");
	}

	return (0);
}

int
robosw_mib(SYSCTL_HANDLER_ARGS)
{
	struct robosw_softc	*sc;
	uint32_t		 value;
	int			 port, metric;

	sc = (struct robosw_softc *) arg1;
	port = (arg2 >> 4);
	metric = (arg2 & 0xF);
	value = sc->hal.api.mib_get(sc, port, metric);
	return (sysctl_handle_16(oidp, NULL, value, req));
}

int
robosw_mib_reset(SYSCTL_HANDLER_ARGS)
{
	struct robosw_softc	*sc;
	uint32_t		 value;
	uint16_t		 t;
	int			 error;

	t = 0;
	CTR1(KTR_DEV,"robosw: mib_reset: 0x%x", req->newptr);
	error = sysctl_handle_16(oidp, &t, 0, req);
	CTR2(KTR_DEV,"robosw: mib_reset: %d - 0x%x", error, req->newptr);
	if ((error != 0) || (req->newptr == NULL))
		return (error);

	CTR0(KTR_DEV,"robosw: mib_reset start");
	sc = (struct robosw_softc *) arg1;
	ROBOSW_RD(GLOBAL_MGMT_CTL,value,sc);
	value |= GLOBAL_MGMT_CTL_RST_MIB;
	ROBOSW_WR(GLOBAL_MGMT_CTL,value,sc);
	value &= ~GLOBAL_MGMT_CTL_RST_MIB;
	ROBOSW_WR(GLOBAL_MGMT_CTL,value,sc);
	CTR0(KTR_DEV,"robosw: mib_reset done");

	return (0);
}

int
robosw_sysctl_dump(SYSCTL_HANDLER_ARGS)
{
	struct robosw_softc	*sc;
	struct sbuf 		 sbuf;
	int			 error, t;

	sc = (struct robosw_softc *) arg1;
	sbuf_new_for_sysctl(&sbuf, NULL, 128, req);
	sbuf_printf(&sbuf, "\n");

#undef	ROBOSWPRINT
#define	ROBOSWPRINT(_dev)		sbuf_printf(&sbuf,

	t = sc->sc_debug;
	sc->sc_debug = 1;
	ROBOSWDUMP(sc, sc->sc_dev);
	sc->sc_debug = t;

	error = sbuf_finish(&sbuf);
	sbuf_delete(&sbuf);
	return (error);
}
