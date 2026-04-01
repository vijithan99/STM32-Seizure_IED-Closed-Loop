/*
 * stim_scheduler.c
 *
 *  Created on: Mar 31, 2026
 *      Author: vijit
 */


#include "stim_scheduler.h"
#include <string.h>
#include <stdlib.h>

RHSConfigParameters on_chip_parameters;
volatile StimSequence sequences[NUM_STIM_SEQUENCES];

static WriteCommandCircularBuffer scheduled_write_commands;
static uint8_t regs_needing_writes[MAX_REGS_NEEDING_WRITES] = {0};
static uint8_t regs_needing_writes_index = 0;
static bool odd_iteration = false;

#ifdef TRADITIONAL_FAST_SETTLE
static const uint8_t amp_settle_reg_address = 10;
#else
static const uint8_t amp_settle_reg_address = 12;
#endif
#ifdef CHARGE_RECOVERY_SWITCH
static const uint8_t charge_recovery_reg_address = 46;
#else
static const uint8_t charge_recovery_reg_address = 48;
#endif
static const uint8_t stim_on_address = 42;
static const uint8_t stim_pol_address = 44;
static const uint8_t compliance_monitor_address = 40;
static const uint8_t pos_stim_mag_address_offset = 96;
static const uint8_t neg_stim_mag_address_offset = 64;


static inline int16_t num_commands_in_buffer(void)
{
	if (scheduled_write_commands.read_index > scheduled_write_commands.write_index) {
		return scheduled_write_commands.write_index + MAX_COMMANDS_IN_BUFFER - scheduled_write_commands.read_index;
	} else {
		return scheduled_write_commands.write_index - scheduled_write_commands.read_index;
	}
}

static void set_last_command_update(void)
{
	uint8_t last_command_write_index;
	if (scheduled_write_commands.write_index == 0) {
		last_command_write_index = MAX_COMMANDS_IN_BUFFER - 1;
	} else {
		last_command_write_index = scheduled_write_commands.write_index - 1;
	}

	scheduled_write_commands.commands[last_command_write_index].update = true;
}


// Append command to bottom of buffer
static void write_command_to_buffer(ScheduledWriteCommand command)
{
	// Report error when attempting to write to a full buffer
	if (num_commands_in_buffer() >= MAX_COMMANDS_IN_BUFFER) {
		handle_error(CommandBufferOverflowError);
	}

	// Add command to current write index.
	scheduled_write_commands.commands[scheduled_write_commands.write_index] = command;

	// Increment index
	scheduled_write_commands.write_index++;

	// Wrap around when end of circular buffer index is reached.
	if (scheduled_write_commands.write_index >= MAX_COMMANDS_IN_BUFFER) {
		scheduled_write_commands.write_index = 0;
	}
}
