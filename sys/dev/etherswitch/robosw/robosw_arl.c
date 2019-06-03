/*-
 * Copyright (c) 2019 Michael Zhilin <mizhka@freebsd.org>
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

#include "robosw_var.h"

int
robosw_arl_entry(device_t dev, etherswitch_atu_entry_t *entry)
{
	struct robosw_softc	*sc;
	struct robosw_arl_entry	*arl_entry;
	int			 tmp;

	sc = device_get_softc(dev);
	arl_entry = STAILQ_FIRST(sc->arl_table);

	for (tmp = 0; tmp < entry->id; tmp++)
		if ((arl_entry = STAILQ_NEXT(arl_entry, next)) == NULL)
			return (ENOENT);

	entry->es_portmask = arl_entry->portmask;
	for (tmp = 0; tmp < ETHER_ADDR_LEN; tmp++)
		entry->es_macaddr[tmp] = arl_entry->macaddr[tmp];

	return (0);
}

void
robosw_arl_free(device_t dev)
{
	struct robosw_softc	*sc;
	struct robosw_api	*api;
	struct robosw_arl_entry *entry, *entry2;

	sc = device_get_softc(dev);
	api = ROBOSW_API_SOFTC(sc);
	entry = STAILQ_FIRST(sc->arl_table);

	while (entry != NULL) {
		entry2 = STAILQ_NEXT(entry, next);
		free(entry, M_BCMSWITCH);
		entry = entry2;
	}
	STAILQ_INIT(sc->arl_table);
}

/*
 * This call should fetch all table into temporary buffer for further access
 * If it's called second time, then it will fetch all buckets again
 */
int
robosw_arl_table(device_t dev, etherswitch_atu_table_t *table)
{
	struct robosw_softc	*sc;
	struct robosw_api	*api;
	struct robosw_arl_entry *entry;

	sc = device_get_softc(dev);
	api = ROBOSW_API_SOFTC(sc);
	robosw_arl_free(dev);

	api->arl_iterator(sc);
	for (; api->arl_next(sc) == 0;) {}

	table->es_nitems = 0;
	STAILQ_FOREACH(entry, sc->arl_table, next) {
		table->es_nitems++;
	}

	return (0);
}

int
robosw_arl_flushall(device_t dev)
{
	struct robosw_softc	*sc;
	struct robosw_api	*api;
	struct robosw_arl_entry	*ent;
	int			 err;

	sc = device_get_softc(dev);
	api = ROBOSW_API_SOFTC(sc);

	if (api->arl_flushent == NULL)
		return (ENOSYS);

	STAILQ_FOREACH(ent, sc->arl_table, next) {
		err = api->arl_flushent(sc, ent);
		if (err != 0)
			return (err);
		}

	return (0);
}
