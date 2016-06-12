/*
 * b53_ddb.c
 *
 *  Created on: Jun 9, 2016
 *      Author: mizhka
 */

#include "opt_ddb.h"

#ifndef DDB
#define DDB
#define KERNEL
#endif

#ifdef DDB
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/queue.h>

#include <ddb/ddb.h>
#include <ddb/db_lex.h>

#include "b53_var.h"
#include "b53_reg.h"

static struct command_table db_b53_table = LIST_HEAD_INITIALIZER(db_t4_table);
_DB_SET(_show, b53, NULL, db_show_table, 0, &db_b53_table);

DB_FUNC(read, db_show_b53read, db_b53_table, CS_OWN, NULL)
{
	device_t	dev;
	uint32_t	reg;
	int		t;

	t = db_read_token();
	if (t == tIDENT) {
		dev = device_lookup_by_name(db_tok_string);
		t = db_read_token();
		if (t == tNUMBER) {
			reg = db_tok_number;
		}
	}

	db_skip_to_eol();
	/*TODO: usage info */
	//	db_printf("asked to read 0x%x from %s\n", reg, device_get_nameunit(dev));
//	db_printf("usage: show b53 read <b53device> <reg>\n");
	db_printf("0x%x: 0x%x\n", reg, b53chip_read4(device_get_softc(dev), reg));
	return;
}

DB_FUNC(write, db_show_b53write, db_b53_table, CS_OWN, NULL)
{
	device_t	dev;
	uint32_t	reg;
	uint32_t	val;

	int		t;

	t = db_read_token();
	if (t == tIDENT) {
		dev = device_lookup_by_name(db_tok_string);
		t = db_read_token();
		if (t == tNUMBER) {
			reg = db_tok_number;
			t = db_read_token();
			if (t == tNUMBER)
				val = db_tok_number;
		}
	}

	db_skip_to_eol();
	/*TODO: usage info */
	//	db_printf("asked to read 0x%x from %s\n", reg, device_get_nameunit(dev));
//	db_printf("usage: show b53 read <b53device> <reg>\n");
	b53chip_write4(device_get_softc(dev), reg, val);
	return;
}

DB_FUNC(dump, db_show_b53dump, db_b53_table, CS_OWN, NULL)
{
	struct b53_softc	*sc;
	device_t		 dev;
	int			 t;


	t = db_read_token();
	if (t == tIDENT) {
		dev = device_lookup_by_name(db_tok_string);
	}

	sc = device_get_softc(dev);

	db_skip_to_eol();
	/*TODO: usage info */
	//	db_printf("asked to read 0x%x from %s\n", reg, device_get_nameunit(dev));
//	db_printf("usage: show b53 read <b53device> <reg>\n");
	B53DUMP();
	return;
}

DB_FUNC(reset, db_show_b53reset, db_b53_table, CS_OWN, NULL)
{
	device_t		 dev;
	int			 t;


	t = db_read_token();
	if (t == tIDENT) {
		dev = device_lookup_by_name(db_tok_string);
	}

	db_skip_to_eol();
	/*TODO: usage info */
	//	db_printf("asked to read 0x%x from %s\n", reg, device_get_nameunit(dev));
//	db_printf("usage: show b53 read <b53device> <reg>\n");
	b53chip_reset(dev);
	return;
}


//static void
//t4_dump_tcb(struct adapter *sc, int tid)
//{
//	uint32_t base, i, j, off, pf, reg, save, tcb_addr, win_pos;
//
//	reg = PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_OFFSET, 2);
//	save = t4_read_reg(sc, reg);
//	base = sc->memwin[2].mw_base;
//
//	/* Dump TCB for the tid */
//	tcb_addr = t4_read_reg(sc, A_TP_CMM_TCB_BASE);
//	tcb_addr += tid * TCB_SIZE;
//
//	if (is_t4(sc)) {
//		pf = 0;
//		win_pos = tcb_addr & ~0xf;	/* start must be 16B aligned */
//	} else {
//		pf = V_PFNUM(sc->pf);
//		win_pos = tcb_addr & ~0x7f;	/* start must be 128B aligned */
//	}
//	t4_write_reg(sc, reg, win_pos | pf);
//	t4_read_reg(sc, reg);
//
//	off = tcb_addr - win_pos;
//	for (i = 0; i < 4; i++) {
//		uint32_t buf[8];
//		for (j = 0; j < 8; j++, off += 4)
//			buf[j] = htonl(t4_read_reg(sc, base + off));
//
//		db_printf("%08x %08x %08x %08x %08x %08x %08x %08x\n",
//		    buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
//		    buf[7]);
//	}
//
//	t4_write_reg(sc, reg, save);
//	t4_read_reg(sc, reg);
//}
//
//static void
//t4_dump_devlog(struct adapter *s)
//{
//	struct devlog_params *dparams = &sc->params.devlog;
//	struct fw_devlog_e e;
//	int i, first, j, m, nentries, rc;
//	uint64_t ftstamp = UINT64_MAX;
//
//	if (dparams->start == 0) {
//		db_printf("devlog params not valid\n");
//		return;
//	}
//
//	nentries = dparams->size / sizeof(struct fw_devlog_e);
//	m = fwmtype_to_hwmtype(dparams->memtype);
//
//	/* Find the first entry. */
//	first = -1;
//	for (i = 0; i < nentries && !db_pager_quit; i++) {
//		rc = -t4_mem_read(sc, m, dparams->start + i * sizeof(e),
//		    sizeof(e), (void *)&e);
//		if (rc != 0)
//			break;
//
//		if (e.timestamp == 0)
//			break;
//
//		e.timestamp = be64toh(e.timestamp);
//		if (e.timestamp < ftstamp) {
//			ftstamp = e.timestamp;
//			first = i;
//		}
//	}
//
//	if (first == -1)
//		return;
//
//	i = first;
//	do {
//		rc = -t4_mem_read(sc, m, dparams->start + i * sizeof(e),
//		    sizeof(e), (void *)&e);
//		if (rc != 0)
//			return;
//
//		if (e.timestamp == 0)
//			return;
//
//		e.timestamp = be64toh(e.timestamp);
//		e.seqno = be32toh(e.seqno);
//		for (j = 0; j < 8; j++)
//			e.params[j] = be32toh(e.params[j]);
//
//		db_printf("%10d  %15ju  %8s  %8s  ",
//		    e.seqno, e.timestamp,
//		    (e.level < nitems(devlog_level_strings) ?
//			devlog_level_strings[e.level] : "UNKNOWN"),
//		    (e.facility < nitems(devlog_facility_strings) ?
//			devlog_facility_strings[e.facility] : "UNKNOWN"));
//		db_printf(e.fmt, e.params[0], e.params[1], e.params[2],
//		    e.params[3], e.params[4], e.params[5], e.params[6],
//		    e.params[7]);
//
//		if (++i == nentries)
//			i = 0;
//	} while (i != first && !db_pager_quit);
//}
#endif
