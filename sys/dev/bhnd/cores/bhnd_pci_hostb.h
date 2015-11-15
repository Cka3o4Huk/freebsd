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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_CORES_PCI_HOSTB_H_
#define _BHND_CORES_PCI_HOSTB_H_

#define	BHND_HOSTB_DEVNAME	"bhnd_hostb"


/*
 * PCI/PCIE Device-Mode Configuration Registers.
 * 
 * = MAJOR CORE REVISIONS =
 * 
 * There have been four revisions to the BAR0/BAR1 memory mappings used
 * in BHND PCI/PCIE bridge cores:
 * 
 * == PCI_V0 ==
 * Applies to:
 * -  PCI (cid=0x804, revision <= 12)
 * BAR size: 8KB
 * Address Map:
 *	[offset+  size]	type	description
 * 	[0x0000+0x1000]	dynamic mapped backplane address space (window 0).
 * 	[0x1000+0x0800]	fixed	SPROM shadow
 * 	[0x1800+0x0800]	fixed	pci core registers
 * 
 * == PCI_V1 ==
 * Applies to:
 * -  PCI (cid=0x804, revision >= 13)
 * -  PCIE (cid=0x820) with ChipCommon (revision <= 31)
 * BAR size: 16KB
 * Address Map:
 *	[offset+  size]	type	description
 *	[0x0000+0x1000]	dynamic	mapped backplane address space (window 0).
 *	[0x1000+0x1000]	fixed	SPROM shadow
 *	[0x2000+0x1000]	fixed	pci/pcie core registers
 *	[0x3000+0x1000]	fixed	chipcommon core registers
 *
 * == PCI_V2 ==
 * Applies to:
 * - PCIE (cid=0x820) with ChipCommon (revision >= 32)
 * BAR size: 16KB
 * Address Map:
 *	[offset+  size]	type	description
 *	[0x0000+0x1000]	dynamic	mapped backplane address space (window 0).
 *	[0x1000+0x1000]	dynamic	mapped backplane address space (window 1).
 *	[0x2000+0x1000]	fixed	pci/pcie core registers
 *	[0x3000+0x1000]	fixed	chipcommon core registers
 *
 * == PCI_V3 ==
 * Applies to:
 * - PCIE Gen 2 (cid=0x83c)
 * BAR size: 32KB?
 * Address Map:
 *	[offset+  size]	type	description
 *	[0x0000+0x1000]	dynamic	mapped backplane address space (window 0).
 *	[0x1000+0x1000]	dynamic	mapped backplane address space (window 1).
 *	[0x2000+0x1000]	fixed	pci/pcie core registers
 *	[0x3000+0x1000]	fixed	chipcommon core registers
 *	[???]
 * 
 * = MINOR CORE REVISIONS =
 * 
 * == PCI/PCIE Cores Revision >= 3 ==
 * - Mapped GPIO CSRs into the PCI config space. Refer to
 *   BHND_PCI_GPIO_*.
 * 
 * == PCI/PCIE Cores Revision >= 14 ==
 * - Mapped the clock CSR into the PCI config space. Refer to
 *   BHND_PCI_CLK_CTL_ST
 * 
 * = Hardware Bugs =
 * == BAR1 ==
 * 
 * The BHND PCI(e) cores hypothetically support an additional memory mapping
 * of the backplane address space via BAR1, but this appears to be subject
 * to a hardware bug in which BAR1 is initially configured with a 4 byte
 * length.
 * 
 * A work-around for this bug may be possible by writing to the PCI core's
 * BAR1 config register (0x4e0), but this requires further research -- I've
 * found three sources for information on the BAR1 PCI core configuration that
 * may be relevant:
 * 	- The QLogix NetXTreme 10GB PCIe NIC seems to use the same PCIE
 * 	  core IP block as is used in other BHND devices. The bxe(4) driver
 * 	  contains example initialization code and register constants
 * 	  that may apply (e.g. GRC_BAR2_CONFIG/PCI_CONFIG_2_BAR2_SIZE).
 * 	- The publicly available Broadcom BCM440X data sheet (440X-PG02-R)
 * 	  appears to (partially) document a Broadcom PCI(e) core that has a
 * 	  seemingly compatible programming model.
 * 	- The Android bcmdhd driver sources include a possible work-around
 *	  implementation (writing to 0x4e0) in dhd_pcie.c
 */

/* Common PCI/PCIE Config Registers */
#define	BHNDB_PCI_SPROM_CONTROL		0x88	/* sprom property control */
#define	BHNDB_PCI_BAR1_CONTROL		0x8c	/* BAR1 region prefetch/burst control */
#define	BHNDB_PCI_INT_STATUS		0x90	/* PCI and other cores interrupts */
#define	BHNDB_PCI_INT_MASK		0x94	/* mask of PCI and other cores interrupts */
#define	BHNDB_PCI_TO_SB_MB		0x98	/* signal backplane interrupts */
#define	BHNDB_PCI_BACKPLANE_ADDR	0xa0	/* address an arbitrary location on the system backplane */
#define	BHNDB_PCI_BACKPLANE_DATA	0xa4	/* data at the location specified by above address */
#define	BHNDB_PCI_CLK_CTL_ST		0xa8	/* pci config space clock control/status (>=rev14) */
#define	BHNDB_PCI_GPIO_IN		0xb0	/* pci config space gpio input (>=rev3) */
#define	BHNDB_PCI_GPIO_OUT		0xb4	/* pci config space gpio output (>=rev3) */
#define	BHNDB_PCI_GPIO_OUTEN		0xb8	/* pci config space gpio output enable (>=rev3) */


/* PCI_V0  */
#define	BHNDB_PCI_V0_BAR0_WIN0_CONTROL	0x80	/* backplane address space accessed by BAR0/WIN0 */
#define	BHNDB_PCI_V0_BAR1_WIN0_CONTROL	0x84	/* backplane address space accessed by BAR1/WIN0. */

#define	BHNDB_PCI_V0_BAR0_SIZE		0x2000	/* 8KB BAR0 */
#define	BHNDB_PCI_V0_BAR0_WIN0_OFFSET	0x0	/* bar0 + 0x0 accesses configurable 4K region of backplane address space */
#define	BHNDB_PCI_V0_BAR0_WIN0_SIZE	0x1000
#define	BHNDB_PCI_V0_BAR0_SPROM_OFFSET	0x1000	/* bar0 + 4K accesses sprom shadow (in pci core) */
#define BHNDB_PCI_V0_BAR0_SPROM_SIZE	0x0800
#define	BHNDB_PCI_V0_BAR0_PCIREG_OFFSET	0x1800	/* bar0 + 6K accesses pci core registers */
#define	BHNDB_PCI_V0_BAR0_PCIREG_SIZE	0x0800

/* PCI_V1 */
#define	BHNDB_PCI_V1_BAR0_WIN0_CONTROL	0x80	/* backplane address space accessed by BAR0/WIN0 */
#define	BHNDB_PCI_V1_BAR1_WIN0_CONTROL	0x84	/* backplane address space accessed by BAR1/WIN0. */

#define	BHNDB_PCI_V1_BAR0_SIZE		0x4000	/* 16KB BAR0 */
#define	BHNDB_PCI_V1_BAR0_WIN0_OFFSET	0x0	/* bar0 + 0x0 accesses configurable 4K region of backplane address space */
#define	BHNDB_PCI_V1_BAR0_WIN0_SIZE	0x1000
#define	BHNDB_PCI_V1_BAR0_SPROM_OFFSET	0x1000	/* bar0 + 4K accesses sprom shadow (in pci core) */
#define BHNDB_PCI_V1_BAR0_SPROM_SIZE	0x1000
#define	BHNDB_PCI_V1_BAR0_PCIREG_OFFSET	0x2000	/* bar0 + 8K accesses pci/pcie core registers */
#define	BHNDB_PCI_V1_BAR0_PCIREG_SIZE	0x1000
#define	BHNDB_PCI_V1_BAR0_CCREGS_OFFSET	0x3000	/* bar0 + 12K accesses chipc core registers */
#define	BHNDB_PCI_V1_BAR0_CCREGS_SIZE	0x1000

/* PCI_V2 */
#define	BHNDB_PCI_V2_BAR0_WIN0_CONTROL	0x80	/* backplane address space accessed by BAR0/WIN0 */
#define	BHNDB_PCI_V2_BAR1_WIN0_CONTROL	0x84	/* backplane address space accessed by BAR1/WIN0. */
#define	BHNDB_PCI_V2_BAR0_WIN1_CONTROL	0xAC	/* backplane address space accessed by BAR0/WIN1 */

#define	BHNDB_PCI_V2_BAR0_SIZE		0x4000	/* 16KB BAR0 */
#define	BHNDB_PCI_V2_BAR0_WIN0_OFFSET	0x0	/* bar0 + 0x0 accesses configurable 4K region of backplane address space */
#define	BHNDB_PCI_V2_BAR0_WIN0_SIZE	0x1000
#define	BHNDB_PCI_V2_BAR0_WIN1_OFFSET	0x1000	/* bar0 + 4K accesses second 4K window */
#define BHNDB_PCI_V2_BAR0_WIN1_SIZE	0x1000
#define	BHNDB_PCI_V2_BAR0_PCIREG_OFFSET	0x2000	/* bar0 + 8K accesses pci/pcie core registers */
#define	BHNDB_PCI_V2_BAR0_PCIREG_SIZE	0x1000
#define	BHNDB_PCI_V2_BAR0_CCREGS_OFFSET	0x3000	/* bar0 + 12K accesses chipc core registers */
#define	BHNDB_PCI_V2_BAR0_CCREGS_SIZE	0x1000

/* PCI_V3 */
#define	BHNDB_PCI_V3_BAR0_WIN0_CONTROL	0x80	/* backplane address space accessed by BAR0/WIN0 */
#define	BHNDB_PCI_V3_BAR1_WIN0_CONTROL	0x84	/* backplane address space accessed by BAR1/WIN0. */
#define BHNDB_PCI_V3_BAR0_WIN1_CONTROL	0x70	/* backplane address space accessed by BAR0/WIN1 */

#define	BHNDB_PCI_V3_BAR0_SIZE		0x8000	/* 32KB BAR0 (?) */
#define	BHNDB_PCI_V3_BAR0_WIN0_OFFSET	0x0	/* bar0 + 0x0 accesses configurable 4K region of backplane address space */
#define	BHNDB_PCI_V3_BAR0_WIN0_SIZE	0x1000
#define	BHNDB_PCI_V3_BAR0_WIN1_OFFSET	0x1000	/* bar0 + 4K accesses second 4K window */
#define BHNDB_PCI_V3_BAR0_WIN1_SIZE	0x1000
#define	BHNDB_PCI_V3_BAR0_PCIREG_OFFSET	0x2000	/* bar0 + 8K accesses pci/pcie core registers */
#define	BHNDB_PCI_V3_BAR0_PCIREG_SIZE	0x1000
#define	BHNDB_PCI_V3_BAR0_CCREGS_OFFSET	0x3000	/* bar0 + 12K accesses chipc core registers */
#define	BHNDB_PCI_V3_BAR0_CCREGS_SIZE	0x1000

/* PCI_INT_STATUS */
#define	BHNDB_PCI_SBIM_STATUS_SERR	0x4	/* backplane SBErr interrupt status */

/* PCI_INT_MASK */
#define	BHNDB_PCI_SBIM_SHIFT		8	/* backplane core interrupt mask bits offset */
#define	BHNDB_PCI_SBIM_MASK		0xff00	/* backplane core interrupt mask */
#define	BHNDB_PCI_SBIM_MASK_SERR	0x4	/* backplane SBErr interrupt mask */

/* PCI_SPROM_CONTROL */
#define	BHNDB_PCI_SPROM_SZ_MSK		0x02	/* SPROM Size Mask */
#define	BHNDB_PCI_SPROM_LOCKED		0x08	/* SPROM Locked */
#define	BHNDB_PCI_SPROM_BLANK		0x04	/* indicating a blank SPROM */
#define	BHNDB_PCI_SPROM_WRITEEN		0x10	/* SPROM write enable */
#define	BHNDB_PCI_SPROM_BOOTROM_WE	0x20	/* external bootrom write enable */
#define	BHNDB_PCI_SPROM_BACKPLANE_EN	0x40	/* Enable indirect backplane access */
#define	BHNDB_PCI_SPROM_OTPIN_USE	0x80	/* device OTP In use */


#endif /* _BHND_CORES_PCI_HOSTB_H_ */