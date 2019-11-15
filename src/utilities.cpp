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
   \file utilities.cpp
   \brief A library with useful functions for handling EtherCAT PDOs and other utilities.
*/

/*****************************************************************************/

#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <string.h>
#include "ecrt.h"
#include "ighm_ros.h"
#include "utilities.h"
#include "ether_ros.h"

namespace utilities
{

int safe_atoi(const char *s, int *val)
{
    long l;
    char *endp;

    l = strtol(s, &endp, 10);
    if (s != endp && *endp == '\0')
    {
        *val = l;
        return 0;
    }
    else
        return -1;
}

bool process_input_bit(uint8_t *data_ptr, uint8_t index, uint8_t subindex)
{
    bool return_value = false;
    uint8_t *new_data_ptr;
    new_data_ptr = (data_ptr + index);
    return_value = EC_READ_BIT(new_data_ptr, subindex);
    return return_value;
}

uint8_t process_input_uint8(uint8_t *data_ptr, uint8_t index)
{
    uint8_t return_value = 0x00;
    uint8_t *new_data_ptr;
    new_data_ptr = (data_ptr + index);
    return_value = EC_READ_U8(new_data_ptr);
    return return_value;
}
int8_t process_input_int8(uint8_t *data_ptr, uint8_t index)
{
    int8_t return_value = 0x00;
    uint8_t *new_data_ptr;
    new_data_ptr = (data_ptr + index);
    return_value = EC_READ_S8(new_data_ptr);
    return return_value;
}

uint16_t process_input_uint16(uint8_t *data_ptr, uint8_t index)
{
    uint16_t return_value = 0x0000;
    uint8_t new_data_ptr[2];
    new_data_ptr[0] = data_ptr[index];
    new_data_ptr[1] = data_ptr[index + 1];
    return_value = EC_READ_U16(new_data_ptr);
    return return_value;
}
int16_t process_input_int16(uint8_t *data_ptr, uint8_t index)
{
    int16_t return_value = 0x0000;
    uint8_t new_data_ptr[2];
    new_data_ptr[0] = data_ptr[index];
    new_data_ptr[1] = data_ptr[index + 1];
    return_value = EC_READ_S16(new_data_ptr);
    return return_value;
}
uint32_t process_input_uint32(uint8_t *data_ptr, uint8_t index)
{
    uint32_t return_value = 0x00000000;
    uint8_t new_data_ptr[4];
    new_data_ptr[0] = data_ptr[index];
    new_data_ptr[1] = data_ptr[index + 1];
    new_data_ptr[2] = data_ptr[index + 2];
    new_data_ptr[3] = data_ptr[index + 3];
    return_value = EC_READ_U32(new_data_ptr);
    return return_value;
}
int32_t process_input_int32(uint8_t *data_ptr, uint8_t index)
{
    int32_t return_value = 0x00000000;
    uint8_t new_data_ptr[4];
    new_data_ptr[0] = data_ptr[index];
    new_data_ptr[1] = data_ptr[index + 1];
    new_data_ptr[2] = data_ptr[index + 2];
    new_data_ptr[3] = data_ptr[index + 3];
    return_value = EC_READ_S32(new_data_ptr);
    return return_value;
}
uint64_t process_input_uint64(uint8_t *data_ptr, uint8_t index)
{
    uint64_t return_value = 0x0000000000000000;
    uint8_t new_data_ptr[8];
    new_data_ptr[0] = data_ptr[index];
    new_data_ptr[1] = data_ptr[index + 1];
    new_data_ptr[2] = data_ptr[index + 2];
    new_data_ptr[3] = data_ptr[index + 3];
    new_data_ptr[4] = data_ptr[index + 4];
    new_data_ptr[5] = data_ptr[index + 5];
    new_data_ptr[6] = data_ptr[index + 6];
    new_data_ptr[7] = data_ptr[index + 7];
    return_value = EC_READ_U64(new_data_ptr);
    return return_value;
}
int64_t process_input_int64(uint8_t *data_ptr, uint8_t index)
{
    int64_t return_value = 0x0000000000000000;
    uint8_t new_data_ptr[8];
    new_data_ptr[0] = data_ptr[index];
    new_data_ptr[1] = data_ptr[index + 1];
    new_data_ptr[2] = data_ptr[index + 2];
    new_data_ptr[3] = data_ptr[index + 3];
    new_data_ptr[4] = data_ptr[index + 4];
    new_data_ptr[5] = data_ptr[index + 5];
    new_data_ptr[6] = data_ptr[index + 6];
    new_data_ptr[7] = data_ptr[index + 7];
    return_value = EC_READ_S64(new_data_ptr);
    return return_value;
}

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC)
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    }
    else
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        ROS_INFO("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        ROS_INFO("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        ROS_INFO("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        ROS_INFO("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        ROS_INFO("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

ssize_t insist_write(int fd, const char *buf, size_t count)
{
    ssize_t ret;
    size_t orig_count = count;

    while (count > 0)
    {
        ret = write(fd, buf, count);
        if (ret < 0)
            return ret;
        buf += ret;
        count -= ret;
    }
    return orig_count;
}

// trim string from start
std::string &ltrim(std::string &str, const std::string &chars)
{
    str.erase(0, str.find_first_not_of(chars));
    return str;
}

std::string &rtrim(std::string &str, const std::string &chars)
{
    str.erase(str.find_last_not_of(chars) + 1);
    return str;
}

std::string &trim(std::string &str, const std::string &chars)
{
    return ltrim(rtrim(str, chars), chars);
}

void copy_process_data_buffer_to_buf(uint8_t * buffer)
{
    pthread_spin_lock(&lock);
    for (int i = 0; i < master_info.slave_count; i++)
    {
        memcpy((buffer + ethercat_slaves[i].slave.get_pdo_out()),
                (process_data_buf + ethercat_slaves[i].slave.get_pdo_out()),
                (size_t)(ethercat_slaves[i].slave.get_pdo_in() - ethercat_slaves[i].slave.get_pdo_out())
            );
    }
    /*
    buffer + ethercat_slaves[i].slave.get_pdo_out()) ----> the starting address of the slave's output pdos in the buffer

    process_data_buf + ethercat_slaves[i].slave.get_pdo_out()) ----> the starting address of the slave's output pdos in the process_data_buf

    (size_t)(ethercat_slaves[i].slave.get_pdo_in() - ethercat_slaves[i].slave.get_pdo_out() ----> size of output pdos of the slave

    */
    pthread_spin_unlock(&lock);
}
} // namespace utilities
