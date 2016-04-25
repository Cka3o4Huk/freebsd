/*
 * chipc_slicer.h
 *
 *  Created on: Apr 25, 2016
 *      Author: mizhka
 */

#pragma once

#include <sys/slicer.h>

int chipc_slicer_flash(device_t dev, struct flash_slice *slices, int *nslices);
