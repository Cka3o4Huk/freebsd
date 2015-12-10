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

#include <dev/bhnd/bhnd_ids.h>
#include <dev/bhnd/bhndreg.h>
#include <dev/bhnd/bhnd.h>

#include "bhndb_hwdata.h"

/*
 * Resource priority specifications shared by all bhndb(4) bridge
 * implementations.
 */

/*
 * Define a bhndb_port_priority table.
 */
#define	BHNDB_PORTS(...)	\
	.ports		= _BHNDB_PORT_ARRAY(__VA_ARGS__),		\
	.num_ports	= (sizeof(_BHNDB_PORT_ARRAY(__VA_ARGS__)) /	\
	    sizeof(_BHNDB_PORT_ARRAY(__VA_ARGS__)[0]))

#define	_BHNDB_PORT_ARRAY(...) (const struct bhndb_port_priority[]) {	\
	__VA_ARGS__							\
}

/*
 * Define a core priority record for all cores matching @p devclass and
 * @p unit.
 * 
 * If a devclass of BHNDB_DEVCLASS_INVALID is specified, this will match
 * on all device classes.
 * 
 * If a unit number of -1 is specified, this will match on all units.
 */
#define	BHNDB_CLASS_PRIO(_devclass, _unit, _priority, ...) {		\
	.match	= {							\
		.vendor	= BHND_MFGID_INVALID,				\
		.device	= BHND_COREID_INVALID,				\
		.hwrev	= { BHND_HWREV_INVALID, BHND_HWREV_INVALID },	\
		.class	= (BHND_DEVCLASS_ ## _devclass),		\
		.unit	= (_unit)					\
	},								\
	.priority = (BHNDB_PRIORITY_ ## _priority),		\
	BHNDB_PORTS(__VA_ARGS__)					\
}

/* Define a port priority record for the (_type, 0, 0) type/port/region
 * triplet. */
#define	BHNDB_PORT0_PRIO(_type, _priority) {			\
	.type		= (BHND_PORT_ ## _type),		\
	.port		= 0,					\
	.region		= 0,					\
	.priority	= (BHNDB_PRIORITY_ ## _priority)	\
}

/**
 * Generic resource priority configuration usable with all currently supported
 * bcma(4) and siba(4)-based PCI devices.
 */
const struct bhndb_hw_priority bhndb_generic_priority_table[] = {
	/*
	 * Ignorable device classes.
	 * 
	 * Runtime access to these cores is not required, and no register
	 * windows should be reserved for these device types.
	 */
	BHNDB_CLASS_PRIO(SOCI,		-1,	NONE),
	BHNDB_CLASS_PRIO(SOCB,		-1,	NONE),
	BHNDB_CLASS_PRIO(EROM,		-1,	NONE),
	BHNDB_CLASS_PRIO(OTHER,		-1,	NONE),

	/*
	 * Low priority device classes.
	 * 
	 * These devices do not sit in a performance-critical path and can be
	 * treated as a low allocation priority.
	 */
	BHNDB_CLASS_PRIO(CC,		-1,	LOW,
		/* Device Block */
		BHNDB_PORT0_PRIO(DEVICE,	LOW),

		/* CC agent registers are not accessed via the bridge. */
		BHNDB_PORT0_PRIO(AGENT,		NONE)
	),

	BHNDB_CLASS_PRIO(PMU,		-1,	LOW,
		/* Device Block */
		BHNDB_PORT0_PRIO(DEVICE,	LOW),

		/* PMU agent registers are not accessed via the bridge. */
		BHNDB_PORT0_PRIO(AGENT,		NONE)
	),

	/*
	 * Default Core Behavior
	 * 
	 * All other cores are assumed to require effecient runtime access to
	 * the default device port, and if supported by the bus, an agent port.
	 */
	BHNDB_CLASS_PRIO(INVALID,	-1,	DEFAULT,
		/* Device Block */
		BHNDB_PORT0_PRIO(DEVICE,	HIGH),

		/* Agent Block */
		BHNDB_PORT0_PRIO(AGENT,		DEFAULT)
	),

	BHNDB_HW_PRIORITY_TABLE_END
};