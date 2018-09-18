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
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
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

/**
   \file utilities.h
   \brief Utilities header file.

   Includes:
   - Functions for processing EtherCAT PDOs
   - Function for insisting write to file
   - Function for safe ascii to integer conversion
   - Function for adding two timespec structs
   - Functions for checking domain and master states
*/

/*****************************************************************************/
/** \fn struct timespec timespec_add(struct timespec time1, struct timespec time2)
    \brief Adds the timespec of the \a time2 to the \a time1 timespec struct.

    The sum is returned in the \a time1 timespec struct.
    \param time1
    \brief The first timespec struct.
    \param time2
    \brief The second timespec struct, to be added to the first one.
*/
/** \fn void check_domain1_state(void)
    \brief Checks the domain1 state variable.

    The checks are for the working counter states and values.

*/
/** \fn check_master_state(void)
    \brief Checks the master state variable.

    The checks are for AL states and slaves responding to the master.
*/
/** \fn bool process_input_bit(uint8_t *data_ptr, uint8_t index, uint8_t subindex)
    \brief Returns bit indexed with \a index,\a subindex of the \a data_ptr buffer.

    \param data_ptr The buffer to get the data
    \param index The byte index in the buffer
    \param subindex The bit index inside the byte, indexed by \a index
*/
/** \fn uint8_t process_input_uint8(uint8_t *data_ptr, uint8_t index)
    \brief Returns unsigned byte indexed with \a index of the \a data_ptr buffer.

    \param data_ptr The buffer to get the data
    \param index The unsigned byte index in the buffer
*/
/** \fn int8_t process_input_sint8(uint8_t *data_ptr, uint8_t index)
    \brief Returns signed byte indexed with \a index of the \a data_ptr buffer.

    \param data_ptr The buffer to get the data
    \param index The signed byte index in the buffer
*/
/** \fn uint16_t process_input_uint16(uint8_t *data_ptr, uint8_t index);
    \brief Returns unsigned 16-bit integer indexed with \a index of the \a data_ptr buffer.

    \param data_ptr The buffer to get the data
    \param index The unsigned 16-bit integer index in the buffer
*/
/** \fn int16_t process_input_sint16(uint8_t *data_ptr, uint8_t index)
    \brief Returns signed 16-bit integer indexed with \a index of the \a data_ptr buffer.

    \param data_ptr The buffer to get the data
    \param index The signed 16-bit integer index in the buffer
*/
/** \fn int32_t process_input_sint32(uint8_t *data_ptr, uint8_t index)
    \brief Returns signed 32-bit integer indexed with \a index of the \a data_ptr buffer.

    \param data_ptr The buffer to get the data
    \param index The signed 32-bit integer index in the buffer
*/
/** \fn ssize_t insist_write(int fd, const char *buf, size_t count)
    \brief Writes to a \a file descriptor, persistently

    The main difference between insist_write() and write() from <unistd.h>,
    is that write() could write less than \a count bytes to the fd, for various
    reasons (see man 3 write). Therefore the need for a persistent write to the
    fd was apparent, hence the insist_write().

    \param fd The file descriptor to write to
    \param buf The buffer to read from
    \param count Number of bytes to write
*/
/** \fn int safe_atoi(const char *s, int *val)
    \brief Converts an ascii sequence (NULL terminated or "escaped") to integer, safely.

    \param s The ascii sequence (string)
    \param val The value to return
*/

/** \fn std::string &ltrim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
    \brief Left trims a string.

    This function trims any character specified in \a chars, which is left of the input
    \a str.

    \param str The input untrimmed string.
    \param chars The characters to trim.
*/
/** \fn std::string &rtrim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
    \brief Right trims a string.

    This function trims any character specified in \a chars, which is right of the input
    \a str.

    \param str The input untrimmed string.
    \param chars The characters to trim.
*/
/** \fn std::string &trim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
    \brief Trims a string from the left and right.

    This function trims any character specified in \a chars, which is right  or left
    of the input \a str. Calls internally the \a ltrim and \a rtrim functions.
    See more at: http://www.martinbroadhurst.com/how-to-trim-a-stdstring.html
    \see ltrim
    \see rtrim
*/

#ifndef UTIL_LIB_H
#define UTIL_LIB_H

#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <string>

namespace utilities
{
bool process_input_bit(uint8_t *data_ptr, uint8_t index, uint8_t subindex);

uint8_t process_input_uint8(uint8_t *data_ptr, uint8_t index);

int8_t process_input_int8(uint8_t *data_ptr, uint8_t index);

uint16_t process_input_uint16(uint8_t *data_ptr, uint8_t index);

int16_t process_input_int16(uint8_t *data_ptr, uint8_t index);

int32_t process_input_int32(uint8_t *data_ptr, uint8_t index);

uint32_t process_input_uint32(uint8_t *data_ptr, uint8_t index);

int64_t process_input_int64(uint8_t *data_ptr, uint8_t index);

uint64_t process_input_uint64(uint8_t *data_ptr, uint8_t index);

ssize_t insist_write(int fd, const char *buf, size_t count);

int safe_atoi(const char *s, int *val);

struct timespec timespec_add(struct timespec time1, struct timespec time2);

void check_domain1_state(void);

void check_master_state(void);

void copy_process_data_buffer_to_buf(uint8_t *buffer);

std::string &ltrim(std::string &str, const std::string &chars = "\t\n\v\f\r ");

std::string &rtrim(std::string &str, const std::string &chars = "\t\n\v\f\r ");

std::string &trim(std::string &str, const std::string &chars = "\t\n\v\f\r ");


} // namespace utilities
#endif /* UTIL_LIB_H */
