/*
 * user_functions.c
 *
 *  Created on: May 28, 2026
 *      Author: vijit


 This file is part of the Intan Technologies RHS STM32 Firmware Framework.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the “Software”), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.


  See <http://www.intantech.com> for documentation and product information.

 */

#include "user_functions.h"
//#include "commonautosequences.h"
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

// Specify condition that should result in the main while loop ending.
// By default, escape once NUMBER_OF_SECONDS_TO_ACQUIRE seconds of data has been gathered.
int loop_escape(void)
{
	// Escape once sample memory capacity (default 1 second of data) has been reached.
#ifdef OFFLINE_TRANSFER
	return sample_counter > per_channel_sample_memory_capacity;
#else
	return 0;
#endif
}


// Write any desired data from this sequence to memory.
// By default, only the result corresponding to a CONVERT on FIRST_SAMPLED_CHANNEL is saved per sequence.
void write_data_to_memory(void)
{
#ifdef OFFLINE_TRANSFER
	// Save single sample to sample_memory array.
	for (int i = 0; i < NUM_SAMPLED_CHANNELS; i++) {
		sample_memory[(sample_counter * NUM_SAMPLED_CHANNELS) + i] = command_sequence_MISO[FIRST_SAMPLED_CHANNEL + i + 2];
	}
	sample_counter++;

//	// Read results of aux command slots (not used in this sample example).
//	// For more advanced programs that require reading of aux command results, those would be read and saved here.
//	uint32_t aux0_result = command_sequence_MISO[18]; // Result of AUX SLOT 1 from this command sequence
//	uint32_t aux1_result = command_sequence_MISO[19]; // Result of AUX SLOT 2 from this command sequence
//	uint32_t aux2_result = command_sequence_MISO[0];  // Result of AUX SLOT 3 from the previous command sequence
//	uint32_t aux3_result = command_sequence_MISO[1];  // Result of AUX SLOT 4 from the previous command sequence
#endif
}

// Determine if data is ready to be transmitted, and if so, transmit (for example via USART).
void transmit_data_realtime(void)
{
#ifndef OFFLINE_TRANSFER
	// By default, do nothing (default example program will only transmit all data at once after acquisition
	// period has finished). So, this function (which is executed once per interrupt routine) should do nothing.

	// If instead, real-time data transfer is desired, user should uncomment the code below.
	// Note that unless loop_escape() is altered, main loop will exit after a period, at which point realtime data
	// transfer will stop. If this is not desired, change loop_escape() so that it never returns 1.


	// IMPORTANT NOTE - Data is written to memory from SPI through DMA, and read from memory to USART through DMA.
	// DMA transmission is automatic, so if it takes too long for USART data to transmit, it's possible for the next sample
	// of data to be writing into memory before the USART read completes. Reading and writing at the same time leads to data corruption.
	// If you uncomment the following code, the data in memory will be overwritten with hardcoded integer values.
	// This allows for obvious detection of corrupted data, as anything transmitted across USART that's not an integer between 0 and
	// CONVERT_COMMANDS_PER_SEQUENCE + AUX_COMMANDS_PER_SEQUENCE will be a result of corruption.
	// Data corruption is more likely to occur with larger NUM_CHANNELS_TO_TRANSMIT, slower USART Baud rate, and faster SPI Baud rate.
//	for (int i = 0; i < CONVERT_COMMANDS_PER_SEQUENCE + AUX_COMMANDS_PER_SEQUENCE; i++) {
//		command_sequence_MISO[i] = i;
//	}
	transmit_dma_to_usart(&command_sequence_MISO[FIRST_SAMPLED_CHANNEL + 2], NUM_SAMPLED_CHANNELS * sizeof(uint32_t));
#endif
}


// Transmit accumulated data after acquisition has finished (for example via USART).
void transmit_data_offline(void)
{
	// This is a relatively large transfer, too much for a single HAL DMA function call.
	// Ideally, we'd do something like:
	//	if (HAL_UART_Transmit(&USART, (uint8_t*) &sample_memory[0], NUM_SAMPLED_CHANNELS * SAMPLES_IN_MEMORY * sizeof(uint32_t), HAL_MAX_DELAY) != HAL_OK)
	//	{
	//		Error_Handler();
	//	}
	// but, 320,000 byte (if NUM_SAMPLED_CHANNELS is 4 and SAMPLES_IN_MEMORY is 20000) transfer too much for a single HAL function call.

	// 4*samples_per_chunk needs to fit into a uint16_t (max value 65535), so the max value of samples_per_chunk
	// is 32767. Ideally, total_samples_in_memory divides into this value cleanly, so 16000 is a reasonable candidate.
	// However, for reasons that are unclear, at high Baud rates, large transfers seem more likely to fail. So, dividing
	// into very small chunks seems to be the most reliable at high Baud rates.

	// We do the same thing for LL, for consistency - optimized performance is not critical for offline transfers, so there is likely
	// no significant downside to chunking data into many smaller transfers.

	const uint16_t samples_per_chunk = 1;
	const uint32_t total_samples_in_memory = NUM_SAMPLED_CHANNELS * calculate_sample_rate() * NUMBER_OF_SECONDS_TO_ACQUIRE;
	const uint32_t num_chunks = floor(total_samples_in_memory / samples_per_chunk);
	const uint16_t remaining_samples = total_samples_in_memory % samples_per_chunk;

	// Transmit multiple complete chunks of data
	for (int i = 0; i < num_chunks; i++) {
		uart_ready = false;
		transmit_dma_to_usart(&sample_memory[samples_per_chunk * i], samples_per_chunk * sizeof(uint32_t));
		while (!uart_ready) {}
	}

	// Transmit any remaining data too small to fit in a complete chunk
	if (remaining_samples > 0) {
		uart_ready = false;
		transmit_dma_to_usart(&sample_memory[samples_per_chunk * num_chunks], remaining_samples * sizeof(uint32_t));
		while (!uart_ready) {}
	}
}


// Handle when a compliance read returns results.
// By default, write Compliance_Monitor pin low if all zeros, high if any non-zeros.
void handle_compliance_result(uint16_t compliance_data)
{
	if ((compliance_data & 0xffff) == 0) {
		// Compliance monitor read all zeros (compliance limit not exceeded) - by default, write pin low
		write_pin(Compliance_Monitor_GPIO_Port, Compliance_Monitor_Pin, false);
	} else {
		// Compliance monitor read at least one 1s (compliance limit exceeded on some channel) - by default, write pin high
		write_pin(Compliance_Monitor_GPIO_Port, Compliance_Monitor_Pin, true);
	}
}
