/*
 * uart_recieve.c
 *
 *  Created on: Mar 10, 2026
 *      Author: vijit
 */

#include "uart_recieve.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"

static input_buffer_t *g_input_buf = NULL;		// Creating the pointer but not pointing it to input_buf incase it has not been created yet
static uint8_t g_rx_byte = 0;

static uint8_t g_sample_bytes[3];
static uint8_t g_sample_byte_index = 0;

void uart_rx_init(input_buffer_t *input_buf){
    g_input_buf = input_buf;
    g_sample_byte_index = 0;
}

void uart_rx_start(void){
    HAL_StatusTypeDef st;
    g_input_buf->uart_call++;
    st = HAL_UART_Receive_IT(&hcom_uart[COM1], &g_rx_byte, 1);

    if (st != HAL_OK) {
        BSP_LED_On(LED_RED);
    }
}

void uart_rx_irq_handler(UART_HandleTypeDef *huart){
    if (huart->Instance == USART3){
        g_sample_bytes[g_sample_byte_index++] = g_rx_byte;

        if (g_sample_byte_index == 3){
            int32_t sample = (int32_t)((g_sample_bytes[2] << 16) | (g_sample_bytes[1] << 8) | g_sample_bytes[0]);
            //buffer_push_sample(g_input_buf, sample);
            if (sample & 0x00800000) {
                    sample |= 0xFF000000;
                }

			buffer_sample_push(g_input_buf, sample);
            g_sample_byte_index = 0;
        }

        g_input_buf->uart_call++;
        HAL_UART_Receive_IT(&hcom_uart[COM1], &g_rx_byte, 1);
    }
}
