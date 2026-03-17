/*
 * seizure_detect.h
 *
 *  Created on: Mar 2, 2026
 *      Author: vijit
 */
#include <stdint.h>

#ifndef SRC_SEIZURE_DETECT_H_
#define SRC_SEIZURE_DETECT_H_

typedef struct {
    int32_t ll;
    float rms;
} stat_features_t;

typedef enum {
	DETECT_NO_EVENT = 0,
	DETECT_SEIZURE_ONSET = 1
} detect_event_t;

typedef struct {
    float k;                  // threshold = mean + k*std
    uint8_t persist_blocks;    // how many consecutive blocks required (e.g., 3)
    uint32_t warmup_blocks;		// Blocks at the beginning of recording that allow for baseline calculation of the data.
    uint32_t refractory_us;    // refractory in microseconds
    float alpha;              // EMA rate for baseline (e.g., 0.01f)
    float min_std;            // floor to avoid tiny std (e.g., 1.0f)
} detect_params_t;

typedef struct {
    detect_params_t p;

    // Baseline estimates of LL
    float mean;
    float var;                // variance estimate

    // Decision state
    uint8_t above_count;
    uint32_t refractory_until_us;
    uint64_t sample_count;		// Final sample of the block

    uint32_t warmup_count;

    // Debug/telemetry
    int32_t last_ll;
    float last_rms;
    float last_thresh;
} detect_state_t;

// Feature Calc
stat_features_t calc_process_block(const int32_t *x, uint32_t n);
static inline void update_line_length(int32_t current, int32_t previous, int32_t *ll);
static inline void update_rms_sum(int32_t sample, int64_t *sum_sq);

// Process Block
void detect_init(detect_state_t *st, const detect_params_t *p);
detect_event_t detect_process_block(detect_state_t *st,
                                    const int32_t *x,
                                    uint32_t n,
                                    uint32_t t_us);

#endif /* SRC_SEIZURE_DETECT_H_ */
