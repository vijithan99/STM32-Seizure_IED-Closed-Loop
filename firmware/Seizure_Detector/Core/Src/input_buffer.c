/*
 * input_buffer.c
 *
 *  Created on: Mar 6, 2026
 *      Author: vijit
 */

#include "input_buffer.h"

void buffer_init(input_buffer_t *input_buf, int32_t *storage, uint16_t fs_hz, uint32_t block_samples){
	input_buf->buffer = storage;
	input_buf->block_samples = block_samples;
	input_buf->fs_hz = fs_hz;
	input_buf->write_index = 0;

	input_buf->ping_ready = 0;
	input_buf->pong_ready = 0;

	input_buf->next_start_sample = 0;
	input_buf->uart_call = 0;
}

void buffer_sample_push(input_buffer_t *input_buf, int32_t sample){
	input_buf->buffer[input_buf->write_index] = sample;
	input_buf->write_index++;

	if (input_buf->write_index == input_buf->block_samples) {
		input_buf->ping_ready = 1;
	}

	else if (input_buf->write_index == 2 * (input_buf->block_samples)){
		input_buf->pong_ready = 1;
		input_buf->write_index = 0;
	}
}

int32_t *buffer_get_ping_ptr(input_buffer_t *input_buf){
    return &input_buf->buffer[0];							// Returns the address of the first buffer position
}

int32_t *buffer_get_pong_ptr(input_buffer_t *input_buf){
    return &input_buf->buffer[input_buf->block_samples];	// Returns the address of the second block position
}

void buffer_clear_ping(input_buffer_t *input_buf){
    input_buf->ping_ready = 0;
}

void buffer_clear_pong(input_buffer_t *input_buf){
    input_buf->pong_ready = 0;
}

