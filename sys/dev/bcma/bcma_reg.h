/*
 * bcma_reg.h
 *
 *  Created on: Jan 24, 2016
 *      Author: mizhka
 */
#pragma once

//TODO: use fdt
#define	BCMA_MEM_RID			0x00
#define	BCMA_EROM_RID			0x01

//Main SOC registers
#define BCMA_CHIPINFO_OFFSET 	0x00
#define BCMA_EROM_OFFSET		0xFC

//Sizes
#define BCMA_CORE_SIZE			0x1000

//EEPROM scanning markers (tags)
#define BCMA_EROM_VALID			0x00000001 /* 0001 */
#define BCMA_EROM_BAD			0xFFFFFFFF /* 1111 1111 1111 1111 1111 1111 1111 1111 */
// Tag values - 3 bits
#define BCMA_EROM_TAG			0x0000000E /* 1110 */
#define BCMA_EROM_TAG_CI		0x00000000 /* 0000 */
#define BCMA_EROM_TAG_MP		0x00000002 /* 0010 MST port */
#define BCMA_EROM_TAG_ADDR		0x00000004 /* 0100 */
#define BCMA_EROM_TAG_END		0x0000000E /* 1110 */

#define BCMA_EROM_TAGX			0x00000006 /* 0110 - we have to ignore 0x8 bit when checking tag for SCAN_ER_TAG_ADDR */

//Masks
#define BCMA_EROM_BITS(value,MASK)		(value & MASK) >> MASK##_SHIFT;
//Fixed
#define BCMA_EROM_CLASS			0x000000F0
#define BCMA_EROM_CLASS_SHIFT	4
#define BCMA_EROM_ID			0x000FFF00
#define BCMA_EROM_ID_SHIFT		8
#define BCMA_EROM_MANUF			0xFFF00000
#define BCMA_EROM_MANUF_SHIFT	20

#define BCMA_EROM_NMP			0x000001F0
#define BCMA_EROM_NMP_SHIFT		4
#define BCMA_EROM_NSP			0x00003E00
#define BCMA_EROM_NSP_SHIFT		9
#define BCMA_EROM_NMW			0x0007C000
#define BCMA_EROM_NMW_SHIFT		14
#define BCMA_EROM_NSW			0x00F80000
#define BCMA_EROM_NSW_SHIFT		17
#define BCMA_EROM_REV			0xFF000000
#define BCMA_EROM_REV_SHIFT		24

//Ports & Wrappers
#define BCMA_EROM_ADDR_AG32		0x00000008
//Fixed
#define BCMA_EROM_ADDR_SZ		0x00000030
#define BCMA_EROM_ADDR_SZ_SHIFT	4
//Address sizes
#define BCMA_EROM_ADDR_SZ_4K	0x00000000
#define BCMA_EROM_ADDR_SZ_8K	0x00000001
#define BCMA_EROM_ADDR_SZ_16K	0x00000002
#define BCMA_EROM_ADDR_SZ_SZD	0x00000003
//Address Types
#define BCMA_EROM_ADDR_TYPE			0x000000C0
#define BCMA_EROM_ADDR_TYPE_SLAVE	0x00000000
#define BCMA_EROM_ADDR_TYPE_BRIDGE	0x00000040
#define BCMA_EROM_ADDR_TYPE_SWRAP	0x00000080
#define BCMA_EROM_ADDR_TYPE_MWRAP	0x000000C0
//Addresses
#define BCMA_EROM_ADDR_PORT			0x00000F00
#define BCMA_EROM_ADDR_PORT_SHIFT	8
#define BCMA_EROM_ADDR_ADDR			0xFFFFF000

#define BCMA_EROM_ADDR_SZ_BASE	0x00001000	/* 4KB */

#define BCMA_EROM_SIZE_SZ_ALIGN	0x00000FFF
#define BCMA_EROM_SIZE_SZ		0xFFFFF000
#define BCMA_EROM_SIZE_SG32		0x00000008
