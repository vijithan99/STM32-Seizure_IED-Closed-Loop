/*
 * uart_recieve.h
 *
 *  Created on: Mar 10, 2026
 *      Author: vijit
 */


#ifndef INC_UART_RECIEVE_H_
#define INC_UART_RECIEVE_H_

#include <stdio.h>
#include <stdint.h>
#include "input_buffer.h"
#include "stm32h7xx_hal.h"

void uart_rx_init(input_buffer_t *input_buf);
void uart_rx_start(void); 				// Waits for UART to recieve info
void uart_rx_irq_handler(UART_HandleTypeDef *huart);

#endif /* INC_UART_RECIEVE_H_ */
