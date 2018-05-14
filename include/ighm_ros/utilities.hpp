
#ifndef UTIL_LIB_HPP
#define UTIL_LIB_HPP

#define SetBit(A,k)     ( A[(k/8)] |= (1 << (k%8)) )
#define ClearBit(A,k)   ( A[(k/8)] &= ~(1 << (k%8)) )


void modify_output_bit (uint8_t * data_ptr, uint8_t index, unsigned int value);

uint16_t process_input_uint16(uint8_t * data_ptr, uint8_t index);

int32_t process_input_int32(uint8_t * data_ptr, uint8_t index, uint8_t subindex);

ssize_t insist_write(int fd, const char *buf, size_t count);

int safe_atoi(const char *s, int *val);

#endif /* UTIL_LIB_HPP */