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

#define BGMAC_REG_DEVICE_CONTROL	0x000
#define BGMAC_REG_DEVICE_STATUS		0x004
#define BGMAC_REG_BIST_STATUS		0x00c
#define BGMAC_REG_INTERRUPT_STATUS	0x020
#define BGMAC_REG_INTERRUPT_MASK	0x024
#define BGMAC_REG_GP_TIMER			0x028

#define BGMAC_REG_PHY_ACCESS				0x180
#define  BGMAC_REG_PHY_ACCESS_DATA  		0x0000ffff
#define  BGMAC_REG_PHY_ACCESS_ADDR			0x001f0000
#define  BGMAC_REG_PHY_ACCESS_ADDR_SHIFT	16
#define  BGMAC_REG_PHY_ACCESS_REG			0x1f000000
#define  BGMAC_REG_PHY_ACCESS_REG_SHIFT		24
#define  BGMAC_REG_PHY_ACCESS_WRITE			0x20000000
#define  BGMAC_REG_PHY_ACCESS_START			0x40000000

#define BGMAC_REG_PHY_CONTROL				0x188
#define  BGMAC_REG_PHY_CONTROL_ADDR			0x0000001f



