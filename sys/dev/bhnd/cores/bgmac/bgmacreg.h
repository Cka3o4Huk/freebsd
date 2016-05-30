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

#pragma once

#define	BGMAC_REG_DEVICE_CONTROL		0x000  //300011
#define	BGMAC_REG_DEVICE_STATUS			0x004  //4110100
#define	BGMAC_REG_BIST_STATUS			0x00c  //0
#define	BGMAC_REG_INTR_STATUS			0x020  //10
#define		BGMAC_REG_INTR_STATUS_ALL	0x0f01fcff
#define		BGMAC_REG_INTR_STATUS_RX	0x00010000
#define		BGMAC_REG_INTR_STATUS_TX1	0x01000000
#define		BGMAC_REG_INTR_STATUS_TX2	0x02000000
#define		BGMAC_REG_INTR_STATUS_TX3	0x04000000
#define		BGMAC_REG_INTR_STATUS_TX4	0x08000000
#define		BGMAC_REG_INTR_STATUS_ERR	0x0000fc00

#define	BGMAC_REG_INTERRUPT_MASK		0x024  //0
#define	BGMAC_REG_GP_TIMER			0x028  //0

#define	BGMAC_REG_INTR_RECV_LAZY			0x100
#define 	BGMAC_REG_INTR_RECV_LAZY_FC_SHIFT	24

#define	BGMAC_REG_CMD_CFG		0x808
#define		BGMAC_REG_CMD_CFG_TX	0x00000001
#define		BGMAC_REG_CMD_CFG_RX	0x00000002
#define 	BGMAC_REG_CMD_CFG_PROM	0x00000010

#define	BGMAC_REG_PHY_ACCESS				0x180
#define		BGMAC_REG_PHY_ACCESS_DATA  		0x0000ffff
#define		BGMAC_REG_PHY_ACCESS_ADDR		0x001f0000
#define		BGMAC_REG_PHY_ACCESS_ADDR_SHIFT		16
#define		BGMAC_REG_PHY_ACCESS_REG		0x1f000000
#define		BGMAC_REG_PHY_ACCESS_REG_SHIFT		24
#define		BGMAC_REG_PHY_ACCESS_WRITE		0x20000000
#define		BGMAC_REG_PHY_ACCESS_START		0x40000000

#define	BGMAC_REG_PHY_CONTROL				0x188
#define		BGMAC_REG_PHY_CONTROL_ADDR			0x0000001f

#define	BGMAC_REG_DMA_RX_BASE				0x220
#define	BGMAC_REG_DMA_RX_CTRL				0x220
#define		BGMAC_REG_DMA_RX_CTRL_ENABLE		0x00000001
#define		BGMAC_REG_DMA_RX_CTRL_FRAMEOFFSET	0x000000FE
#define		BGMAC_REG_DMA_RX_CTRL_PIOMODE		0x00000100
#define		BGMAC_REG_DMA_RX_CTRL_SRXHDENABLE	0x00000200 /* Separate RX Header Descriptor Enable */
#define		BGMAC_REG_DMA_RX_CTRL_OVERFLOW_CONT	0x00000400
#define		BGMAC_REG_DMA_RX_CTRL_DISABLE_PARITYCHK	0x00000800
#define		BGMAC_REG_DMA_RX_CTRL_ADDR_EXTENSION	0x00030000

#define	BGMAC_REG_DMA_RX_INDEX				0x224
#define	BGMAC_REG_DMA_RX_RINGLOW			0x228
#define	BGMAC_REG_DMA_RX_RINGHIGH			0x22C
#define	BGMAC_REG_DMA_RX_STATE				0x230
#define		BGMAC_REG_DMA_RX_STATE_CURDESC		0x00001FFF
#define		BGMAC_REG_DMA_RX_STATE_ST		0xF0000000
#define			BGMAC_REG_DMA_RX_STATE_DISABLED	0
#define			BGMAC_REG_DMA_RX_STATE_ACTIVE	1
#define			BGMAC_REG_DMA_RX_STATE_IDLE	2
#define			BGMAC_REG_DMA_RX_STATE_STOPPED	3
#define			BGMAC_REG_DMA_RX_STATE_SUSPEND	4

#define	BGMAC_REG_DMA_RX_ERROR				0x234
#define		BGMAC_REG_DMA_RX_ERROR_CURDESC		0x0001FFFF
#define		BGMAC_REG_DMA_RX_ERROR_ERRS		0xF0000000
#define			BGMAC_REG_DMA_RX_ERROR_NO	0
#define			BGMAC_REG_DMA_RX_ERROR_ADDRESS	0
#define			BGMAC_REG_DMA_RX_ERROR_OVERFLOW	2
#define			BGMAC_REG_DMA_RX_ERROR_TRANSFER	3
#define			BGMAC_REG_DMA_RX_ERROR_DESCREAD	4
#define			BGMAC_REG_DMA_RX_ERROR_CORE	5
