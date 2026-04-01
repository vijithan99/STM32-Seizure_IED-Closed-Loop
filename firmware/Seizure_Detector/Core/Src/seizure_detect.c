/*
 * seizure_detect.c
 *
 *  Created on: Mar 2, 2026
 *      Author: vijit
 */
#include <math.h>
#include <seizure_detect.h>

static inline float fmaxf_local(float a, float b) { return (a > b) ? a : b;}

static inline void update_line_length(int32_t current, int32_t previous, int32_t *ll){
    int32_t diff = current - previous;

    if (diff < 0) diff = -diff;
    *ll += diff;
}

static inline void update_rms_sum(int32_t sample, int64_t *sum_sq){
    *sum_sq += (int64_t)sample * sample;
}

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

void detect_init(detect_state_t *st, const detect_params_t *p){
    if (!st || !p) return;

    st->p = *p;

    st->ll_mean = 0.0f;
    st->ll_var = 0.0f;

    st->rms_mean = 0.0f;
	st->rms_var = 0.0f;

    st->ll_above_count = 0;
    st->rms_above_count = 0;

    st->refractory_until_us = 0;
    st->sample_count = 0;

    st->warmup_count = 0;

    st->last_ll = 0;
    st->last_rms = 0;
    st->last_ll_thresh = 0.0f;
    st->last_rms_thresh = 0.0f;
}

detect_event_t detect_process_block(detect_state_t *st, const int32_t *x, uint32_t n, uint32_t t_us)
{
    if (!st || !x) return DETECT_NO_EVENT;

    // Track total samples processed
    st->sample_count += n;

    // Refractory gate
    if (t_us < st->refractory_until_us) {
        return DETECT_NO_EVENT;
    }

    // Compute block features
    stat_features_t ft = calc_process_block(x, n);
    float ll = (float)ft.ll;
    float rms = ft.rms;

    st->last_ll = ft.ll;
    st->last_rms = rms;

    // Current thresholds
    float ll_std = sqrtf(fmaxf_local(st->ll_var, st->p.min_std * st->p.min_std));
    float rms_std = sqrtf(fmaxf_local(st->rms_var, st->p.min_std * st->p.min_std));

    float ll_thresh = st->ll_mean + st->p.k_ll * ll_std;
    float rms_thresh = st->rms_mean + st->p.k_rms * rms_std;

    st->last_ll_thresh = ll_thresh;
    st->last_rms_thresh = rms_thresh;

    float a = st->p.alpha;

    // Warmup: update both baselines, no detections
    if (st->warmup_count < st->p.warmup_blocks) {
        float ll_delta = ll - st->ll_mean;
        st->ll_mean += a * ll_delta;
        st->ll_var = (1.0f - a) * (st->ll_var + a * ll_delta * ll_delta);

        float rms_delta = rms - st->rms_mean;
        st->rms_mean += a * rms_delta;
        st->rms_var = (1.0f - a) * (st->rms_var + a * rms_delta * rms_delta);

        st->warmup_count++;
        return DETECT_NO_EVENT;
    }

    // Per-block threshold crossing states
    uint8_t ll_above = (ll >= ll_thresh);
    uint8_t rms_above = (rms >= rms_thresh);

    // Update baselines only when each feature is below its own threshold
    if (!ll_above) {
        float ll_delta = ll - st->ll_mean;
        st->ll_mean += a * ll_delta;
        st->ll_var = (1.0f - a) * (st->ll_var + a * ll_delta * ll_delta);
    }

    if (!rms_above) {
        float rms_delta = rms - st->rms_mean;
        st->rms_mean += a * rms_delta;
        st->rms_var = (1.0f - a) * (st->rms_var + a * rms_delta * rms_delta);
    }

    // Persistence counters for LL
    if (ll_above) {
        if (st->ll_above_count < 255) st->ll_above_count++;
    } else {
        st->ll_above_count = 0;
    }

    // Persistence counters for RMS
    if (rms_above) {
        if (st->rms_above_count < 255) st->rms_above_count++;
    } else {
        st->rms_above_count = 0;
    }

    // Decide event type
    uint8_t ll_persisted = (st->ll_above_count >= st->p.persist_blocks);
    uint8_t rms_persisted = (st->rms_above_count >= st->p.persist_blocks);

    if (ll_persisted && rms_persisted) {
        st->ll_above_count = 0;
        st->rms_above_count = 0;
        st->refractory_until_us = t_us + st->p.refractory_us;
        return DETECT_SEIZURE_ONSET_BOTH;
    }

    if (ll_persisted) {
        st->ll_above_count = 0;
        //st->refractory_until_us = t_us + st->p.refractory_us;
        return DETECT_SEIZURE_ONSET_LL;
    }

    if (rms_persisted) {
        st->rms_above_count = 0;
        //st->refractory_until_us = t_us + st->p.refractory_us;
        return DETECT_SEIZURE_ONSET_RMS;
    }

    return DETECT_NO_EVENT;
}


