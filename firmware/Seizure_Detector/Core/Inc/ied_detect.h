/*
 * ied_detect.h
 *
 *  Created on: May 28, 2026
 *      Author: vijit
 */

#ifndef INC_IED_DETECT_H_
#define INC_IED_DETECT_H_

#include <stdint.h>

typedef enum {
	IED_NO_EVENT = 0,
	IED_DETECTED = 1,
	IED_REJECTED_LOW_AMPLITUDE = 2,
	IED_REJECTED_ARTIFACT = 3,
	IED_IN_REFRACTORY = 4

} detect_ied_event_t;

typedef struct {
    float fs_hz;

    // Threshold parameters
    float env_k;              // e.g. 5.0f
    float amp_min_k;          // e.g. 10.0f
    float amp_artifact_k;     // e.g. 100.0f

    float baseline_alpha;     // e.g. 0.001f
    float min_std;            // e.g. 1.0f

    uint32_t warmup_samples;  // e.g. 10 seconds * fs_hz
    uint32_t refractory_us;   // e.g. 3000000 us = 3 seconds

    uint32_t envelope_window_samples; // e.g. 10 ms worth of samples
} ied_params_t;

typedef struct {
    ied_params_t p;

    uint64_t sample_count;
    uint64_t refractory_until_us;

    // Baselines
    float env_mean;
    float env_var;

    float amp_mean;
    float amp_var;

    // Last values for debugging
    float last_bandpass;
    float last_envelope;
    float last_amp_hp;
    float last_env_thresh;
    float last_amp_min_thresh;
    float last_amp_artifact_thresh;

    // Moving average envelope buffer
    float env_sum;
    uint32_t env_index;
    uint32_t env_count;
    float env_buf[256];   // must be >= envelope_window_samples

    // IIR filter states
    float bp_x1, bp_x2;
    float bp_y1, bp_y2;

    float hp_x1, hp_y1;

} ied_state_t;

void ied_init(ied_state_t *st, const ied_params_t *p);

detect_ied_event_t ied_process_sample(ied_state_t *st,
                               int32_t raw_sample,
							   uint64_t t_us);



#endif /* INC_IED_DETECT_H_ */
