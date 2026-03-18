/*
 * seizure_detect.c
 *
 *  Created on: Mar 2, 2026
 *      Author: vijit
 */
#include <math.h>
#include <seizure_detect.h>

static inline float fmaxf_local(float a, float b) { return (a > b) ? a : b;}

stat_features_t calc_process_block(const int32_t *x, uint32_t n){
    stat_features_t f = {0};

    if (!x || n < 2) return f;

    int32_t ll = 0;
    int64_t sum_sq = 0;

    for (uint32_t i = 0; i < n; i++) {
        update_rms_sum(x[i], &sum_sq);

        if (i > 0) {
            update_line_length(x[i], x[i - 1], &ll);
        }
    }

    f.ll = ll;
    f.rms = sqrtf((float)sum_sq / (float)n);

    return f;
}

static inline void update_line_length(int32_t current, int32_t previous, int32_t *ll){
    int32_t diff = current - previous;

    if (diff < 0) diff = -diff;
    *ll += diff;
}

static inline void update_rms_sum(int32_t sample, int64_t *sum_sq){
    *sum_sq += (int64_t)sample * sample;
}

void detect_init(detect_state_t *st, const detect_params_t *p){
    if (!st || !p) return;

    st->p = *p;

    st->mean = 0.0f;
    st->var = 0.0f;

    st->above_count = 0;
    st->refractory_until_us = 0;
    st->sample_count = 0;

    st->warmup_count = 0;

    st->last_ll = 0;
    st->last_rms = 0;
    st->last_thresh = 0.0f;
}

detect_event_t detect_process_block(detect_state_t *st, const int32_t *x, uint32_t n, uint32_t t_us){
	if (!st || !x) return DETECT_NO_EVENT;

	// Update the final sample being used for the analysis
    st->sample_count += n;

    // Refractory gate
    if (t_us < st->refractory_until_us) {
        return DETECT_NO_EVENT;
    }

    // Compute feature
    stat_features_t ft = calc_process_block(x, n);
    int32_t ll_i32 = ft.ll;
    st->last_ll = ll_i32;

    float ll = (float)ll_i32;

    // Update baseline mean/var using EMA, but only when not strongly above threshold.
    // First compute current std (with floor)
    float std = sqrtf(fmaxf_local(st->var, st->p.min_std * st->p.min_std));
    float thresh = st->mean + st->p.k * std;
    st->last_thresh = thresh;

    // If in early stages of detection this will update the parameters without looking for seizures.
    if (st->warmup_count < st->p.warmup_blocks) {
        // update baseline
        float a = st->p.alpha;
        float delta = ll - st->mean;
        st->mean += a * delta;
        st->var = (1.0f - a) * (st->var + a * delta * delta);

        st->warmup_count++;
        return DETECT_NO_EVENT;
    }

    // Baseline update rule: only adapt when below threshold (prevents poisoning baseline)
    if (ll < thresh) {
        float a = st->p.alpha;
        float delta = ll - st->mean;
        st->mean += a * delta;
        // EMA variance update
        st->var = (1.0f - a) * (st->var + a * delta * delta);
    }

    // Persistence logic - if the line length is above the threshold increase the count
    // When it is above the count for a while then "seizure is detected"
    if (ll >= thresh) {
        if (st->above_count < 255) st->above_count++;
    } else {
        st->above_count = 0;
    }

    if (st->above_count >= st->p.persist_blocks) {
        st->above_count = 0;
        st->refractory_until_us = t_us + st->p.refractory_us;
        return DETECT_SEIZURE_ONSET;
    }

    return DETECT_NO_EVENT;
}


