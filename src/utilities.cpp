
/*
 * utilities.c
 * 
 * A library with useful functions
 * for communication and handling process data, using EtherCAT
 * 
 */

#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include "utilities.hpp"


int safe_atoi(const char *s, int *val)
{
	long l;
	char *endp;

	l = strtol(s, &endp, 10);
	if (s != endp && *endp == '\0') {
		*val = l;
		return 0;
	} else
		return -1;
}

void modify_output_bit (uint8_t * data_ptr, uint8_t index, unsigned int value)
{

    // printf("Blue LED index is: %d\n",index);
    if(value){
        SetBit(data_ptr,index);
        index++;
        SetBit(data_ptr,index);
    }
    else{
        ClearBit(data_ptr,index);
        index++;
        ClearBit(data_ptr,index);
    }
    // printf("Red LED index is: %d\n",index);

}
uint16_t process_input_uint16(uint8_t * data_ptr, uint8_t index)
{
    // uint8 * data_ptr;
    uint16_t return_value = 0x0000;
    // data_ptr = ec_slave[slave_no].inputs;
    /* Move pointer to correct module index*/
    // data_ptr += module_index * 2;
    return_value = data_ptr[index+1];
    return_value = return_value << 8;
    return_value |= data_ptr[index];
//    return_value = ((data_ptr[index]>>8)&0xFFFF) | (data_ptr[index+1]<<8&0xFFFF);
//    printf("Return Value is: %d\n",return_value);
    return return_value;
}
int32_t process_input_int32(uint8_t * data_ptr, uint8_t index, uint8_t subindex)
{
    int32_t return_value = 0x00000000;
    uint16_t * ptr_16 = (uint16_t *)data_ptr;
    uint32_t first_val = 0x00000000;
    first_val |= ptr_16[subindex];
    first_val = first_val << 16;
    first_val |= ptr_16[index];
    // // return_value = (int32_t) ntohl((uint32_t) first_val);
    return_value = first_val;
    printf("Index is: %d and subindex is: %d\n",index,subindex);
    // uint16_t first_val = 0x0000,second_val=0x0000;
    // first_val = ntohs(ptr_16[index]);
    // second_val = ntohs(ptr_16[subindex]);
    // return_value = second_val;
    // return_value <<= 16;
    // return_value |= first_val;
//    return_value = ((data_ptr[index]>>8)&0xFFFF) | (data_ptr[index+1]<<8&0xFFFF);
//    printf("Return Value is: %d\n",return_value);
    return return_value;
}

ssize_t insist_write(int fd, const char *buf, size_t count)
{
    ssize_t ret;
    size_t orig_count = count;

    while (count > 0) {
        ret = write(fd, buf, count);
        if(ret < 0)
            return ret;
        buf += ret;
        count -= ret;

    }
    return orig_count;
}