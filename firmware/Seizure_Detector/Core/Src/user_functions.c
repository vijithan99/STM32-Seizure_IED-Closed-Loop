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
#include "stm32h7xx_nucleo.h"

#define STREAM_CHANNEL_A       8U
#define STREAM_CHANNEL_B       9U
/*
 * SPI acquisition = 5000 Hz.
 * Send every fifth sample for a 1000 Hz UART plot.
 */
#define UART_STREAM_DECIMATION 1U

typedef struct __attribute__((packed))
{
    uint16_t sync;
    uint16_t sequence;
    int16_t channel_a;
    int16_t channel_b;
} uart_stream_packet_t;

static uart_stream_packet_t uart_packet;
static uint16_t uart_sequence = 0U;

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

static int16_t extract_ac_counts(uint8_t convert_slot){
    /*
     * +2 compensates for the RHS2116 two-command pipeline.
     * AC ADC result occupies bits 31:16.
     */
    uint32_t result = command_sequence_MISO[convert_slot + 2U];
    uint16_t raw_ac = (uint16_t)(result >> 16);

    /* Default RHS configuration uses unsigned ADC output. */
    return (int16_t)((int32_t)raw_ac - 32768);
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
    static uint8_t decimation_counter = 0U;

    decimation_counter++;

    if (decimation_counter < UART_STREAM_DECIMATION) {
        return;
    }

    decimation_counter = 0U;

    /*
     * Do not overwrite uart_packet while the previous interrupt-driven
     * UART transmission is still using it.
     */
    if (!uart_ready) {
        return;
    }

    uart_packet.sync      = 0xA55AU;
    uart_packet.sequence  = uart_sequence++;
    uart_packet.channel_a = extract_ac_counts(STREAM_CHANNEL_A);
    uart_packet.channel_b = extract_ac_counts(STREAM_CHANNEL_B);

    uart_ready = false;

    /*
     * Interrupt-driven UART is sufficient for this 1 kHz test and does
     * not require adding USART TX DMA in CubeMX.
     */
    if (HAL_UART_Transmit_IT(
            &hcom_uart[COM1],
            (uint8_t *)&uart_packet,
            sizeof(uart_packet)) != HAL_OK) {

        uart_ready = true;
    }

//	for (int i = 0; i < CONVERT_COMMANDS_PER_SEQUENCE + AUX_COMMANDS_PER_SEQUENCE; i++) {
//		command_sequence_MISO[i] = i;
//	}
//	transmit_dma_to_usart(&command_sequence_MISO[FIRST_SAMPLED_CHANNEL + 2], NUM_SAMPLED_CHANNELS * sizeof(uint32_t));
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

// Configure and transmit register values.
// Initial register values default to the same default settings in the RHX software.
// Any desired changes to these values added after the 'write_initial_reg_values()' function call.
void configure_registers(void)
{
	write_initial_reg_values(&parameters);

	/* Make any changes that differ from defaults here. For example, configure register 2 for impedance check: */

//	// Reg 2: Set zcheck_DAC_power, zcheck_en, zcheck_scale
//	parameters.zcheck_DAC_power = true;
//	set_zcheck_scale(&parameters, ZcheckCs1pF);
//	parameters.zcheck_en = true;
//	set_zcheck_channel(&parameters, FIRST_SAMPLED_CHANNEL);
//	write_command(2, get_register_value(&parameters, 2), false, false);

	// Reg 3: (Actual DAC value which changes over time - instead of setting once here, this should be written sample-by-sample in an aux command list).

	on_chip_parameters = parameters;
}

// Configure the CONVERT commands that are loaded at the beginning of command_sequence_MOSI.
// By default, channels from 0 to CONVERT_COMMANDS_PER_SEQUENCE - 1 (0 to 15) are loaded consecutively (0, 1, 2, 3, ... 15).
void configure_convert_commands(void)
{
	// If default ordering of channel CONVERT commands (0, 1, 2, 3, ... 15) is desired, pass a NULL 2nd parameter to create_convert_sequence().
	create_convert_sequence(NULL);

	// If a custom ordering of channel CONVERT commands is instead desired, create a uint8_t array of size CONVERT_COMMANDS_PER_SEQUENCE
	// and populate each entry with the desired channel number. Then pass this array as the 2nd parameter to create_convert_sequence().
	// For example, if sampling in descending order from CONVERT_COMMANDS_PER_SEQUENCE - 1 (15 to 0) is desired:
	//	uint8_t channel_numbers[CONVERT_COMMANDS_PER_SEQUENCE] = {0};
	//	for (int i = 0; i < CONVERT_COMMANDS_PER_SEQUENCE; i++) {
	//		channel_numbers[i] = (CONVERT_COMMANDS_PER_SEQUENCE - 1) - i;
	//	}
	//	create_convert_sequence(channel_numbers);
}


// Configure the AUX commands that are loaded at the end of command_sequence_MOSI.
// By defaults, command lists from 0 to AUX_COMMANDS_PER_SEQUENCE - 1 (0 to 3) are loaded consecutively (16, 17, 18, 19).
void configure_aux_commands(void)
{
	  // All create_command_list functions return -1 to indicate failure.
	  // Additionally, they should all be used to create command lists of length AUX_COMMAND_LIST_LENGTH, except
	  // for create_command_list_zcheck_DAC. This function returns a command list with a length that depends on the
	  // desired frequency, so if using this command list it's important to set zcheck_DAC_command_slot_position to 0, 1, 2, or
	  // 3 (one of the 4 command slots) to indicate its position, and set zcheck_DAC_command_list_length so that during
	  // execution of this list, after the length has been reached it can begin at 0 again.

	// Slot 0: Write RHS register loading to aux_command_list[0], so that the register values saved in software (parameters) are continually re-written.
	create_command_list_RHS_register_config(&parameters, (uint32_t*) aux_command_list[0], false, AUX_COMMAND_LIST_LENGTH);

	// Slot 1: Write dummy reads to aux_command_list[1], so that register 40 is repeatedly read.
	create_command_list_dummy(&parameters, (uint32_t*) aux_command_list[1], AUX_COMMAND_LIST_LENGTH, read_command(251, false, false));

	// Slot 2: Write dummy reads to aux_command_list[2], so that register 41 is repeatedly read.
	create_command_list_dummy(&parameters, (uint32_t*) aux_command_list[2], AUX_COMMAND_LIST_LENGTH, read_command(252, false, false));

	// Slot 3: Write dummy reads to aux_command_list[3], so that register 42 is repeatedly read.
	create_command_list_dummy(&parameters, (uint32_t*) aux_command_list[3], AUX_COMMAND_LIST_LENGTH, read_command(253, false, false));

	// NOTE: If an impedance check command list is desired, it is created and used slightly differently, because its length is not AUX_COMMAND_LIST_LENGTH
	// but rather depends on impedance test signal frequency. For this demonstration, a zcheck_DAC command list is created and populates aux command slot 3.
	// In this case, the above creation of a dummy command on slot 3 would be redundant and should be commented out.

	// Write impedance check DAC control to aux_command_list[3], so that a sine wave is approximated by the DAC.
	// Note that, as opposed to all other command lists which should be AUX_COMMAND_LIST_LENGTH long, these
	// zcheck_DAC commands can have different lengths depending on desired frequency. To handle this, be sure to:
	// a) assign create_command_list_zcheck_DAC()'s return value to zcheck_DAC_command_list_length, and
	// b) assign which command slot the zcheck_DAC command list is in to zcheck_DAC_command_slot_position.
	// In order to use this, uncomment the declarations of the variables below, located in rhsinterface.h
//	zcheck_DAC_command_list_length = create_command_list_zcheck_DAC(parameters, (uint32_t*) aux_command_list[3], 1000.0, 100);
//	zcheck_DAC_command_slot_position = 3;
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
