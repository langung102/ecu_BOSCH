/*
 * global.h
 *
 *  Created on: Aug 19, 2023
 *      Author: minhl
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#include "main.h"

#define MAX_BUFFER_SIZE 30

extern uint8_t temp;
extern uint8_t index_buffer;
extern uint8_t buffer_flag;
extern uint8_t str[20];

extern uint8_t data_flag1;
extern uint8_t data_flag2;
extern uint8_t dataCounter;

extern SPI_HandleTypeDef hspi1;

#endif /* INC_GLOBAL_H_ */
