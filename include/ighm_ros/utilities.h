
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
