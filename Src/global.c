/*
 * global.c
 *
 *  Created on: Aug 19, 2023
 *      Author: minhl
 */

#include "global.h"

uint8_t temp = 0;
uint8_t index_buffer = 0;
uint8_t buffer_flag = 0;
uint8_t str[20] = "GOATMESSI";

uint8_t data_flag1  = 0;
uint8_t data_flag2  = 0;
uint8_t dataCounter = 0;

SPI_HandleTypeDef hspi1;
