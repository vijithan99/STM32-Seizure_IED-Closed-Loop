/*
 * input_buffer.h
 *
 *  Created on: Mar 6, 2026
 *      Author: vijit
 */
#include <stdint.h>

#ifndef SRC_INPUT_BUFFER_H_
#define SRC_INPUT_BUFFER_H_

#define BLOCK_SAMPLES 500
//static int16_t dma_buffer[2 * BLOCK_SAMPLES];

typedef struct {
    int32_t *buffer;
    uint32_t block_samples;
    uint16_t fs_hz;
    uint32_t write_index;

    volatile uint8_t ping_ready;	// Volatile meaning they will
    volatile uint8_t pong_ready;

    volatile uint32_t overrun_count;

    uint64_t next_start_sample;
    volatile uint32_t uart_call;
} input_buffer_t;

void buffer_init(input_buffer_t *input_buf, int32_t *storage, uint16_t fs_hz, uint32_t block_samples);
void buffer_sample_push(input_buffer_t *input_buf, int32_t sample);

int32_t *buffer_get_ping_ptr(input_buffer_t *input_buf);
int32_t *buffer_get_pong_ptr(input_buffer_t *input_buf);

void buffer_clear_ping(input_buffer_t *input_buf);
void buffer_clear_pong(input_buffer_t *input_buf);

#endif /* SRC_INPUT_BUFFER_H_ */
