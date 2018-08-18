/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2018 Mike Karamousadakis, NTUA CSL
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 3
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace library. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: mkaramousadakis@zoho.eu
 *****************************************************************************/


#ifndef UTIL_LIB_H
#define UTIL_LIB_H

#define SetBit(A, k) (A[(k / 8)] |= (1 << (k % 8)))
#define ClearBit(A, k) (A[(k / 8)] &= ~(1 << (k % 8)))

#include <stdint.h>
#include <stdlib.h>

bool process_input_bit(uint8_t *data_ptr, uint8_t index, uint8_t subindex);

uint8_t process_input_uint8(uint8_t *data_ptr, uint8_t index);

int8_t process_input_sint8(uint8_t *data_ptr, uint8_t index);

uint16_t process_input_uint16(uint8_t *data_ptr, uint8_t index);

int16_t process_input_sint16(uint8_t *data_ptr, uint8_t index);

int32_t process_input_sint32(uint8_t *data_ptr, uint8_t index);

ssize_t insist_write(int fd, const char *buf, size_t count);

int safe_atoi(const char *s, int *val);

#endif /* UTIL_LIB_H */
