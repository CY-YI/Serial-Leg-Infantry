#ifndef SERIAL_TEST_H
#define SERIAL_TEST_H
#include "struct_typedef.h"


#define MAX_TX_LEN  64

extern void Serial_test_int(void);
extern void usart1_send_gyro_init(void);
extern uint16_t pack_gyro_data_to_buf(uint8_t *tx_buf);
#endif
