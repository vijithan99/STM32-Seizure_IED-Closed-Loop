/*
 * ied_detect.c
 *
 *  Created on: May 28, 2026
 *      Author: vijit
 */

#include "ied_detect.h"
#include <math.h>
#include <string.h>

static inline float fmaxf_local(float a, float b)
{
    return (a > b) ? a : b;
}

static float iir_bandpass(ied_state_t *st, float x)
{
    /* PLACEHOLDER coefficients
	 * Below coefficients are calculated using Python's SciPy for a pass band of 50-100 Hz
	 *
	 * The power calculation will be taken from this component rectified
	 */

    const float b0 = 1.0f;
    const float b1 = -2.0f;
    const float b2 = 1.0f;
    const float a1 = -1.983f;
    const float a2 = 0.987f;

    float y = b0 * x + b1 * st->bp_x1 + b2 * st->bp_x2
              - a1 * st->bp_y1 - a2 * st->bp_y2;

    st->bp_x2 = st->bp_x1;
    st->bp_x1 = x;

    st->bp_y2 = st->bp_y1;
    st->bp_y1 = y;

    return y;
}

static float iir_highpass_15hz(ied_state_t *st, float x)
{
    /*
     * alpha = RC / (RC + dt)
     * RC = 1 / (2*pi*fc)
     *
     * This 15 Hz filter will be used for a broadband amplitude increase measurement
     */
    const float fc = 15.0f;
    float dt = 1.0f / st->p.fs_hz;
    float rc = 1.0f / (2.0f * 3.14159265f * fc);
    float alpha = rc / (rc + dt);

    float y = alpha * (st->hp_y1 + x - st->hp_x1);

    st->hp_x1 = x;
    st->hp_y1 = y;

    return y;
}

static float moving_average_envelope(ied_state_t *st, float rectified)
{
    uint32_t win = st->p.envelope_window_samples;

    if (win > 256) {
        win = 256;
    }

    if (st->env_count < win) {
        st->env_buf[st->env_index] = rectified;
        st->env_sum += rectified;
        st->env_count++;

    } else {
        st->env_sum -= st->env_buf[st->env_index];
        st->env_buf[st->env_index] = rectified;
        st->env_sum += rectified;
    }

    st->env_index++;
    if (st->env_index >= win) {
        st->env_index = 0;
    }

    if (st->env_count == 0) {
        return 0.0f;
    }

    return st->env_sum / (float)st->env_count;
}

static void update_baseline(float x, float alpha, float *mean, float *var)
{
    float delta = x - *mean;
    *mean += alpha * delta;
    *var = (1.0f - alpha) * (*var + alpha * delta * delta);
}

void ied_init(ied_state_t *st, const ied_params_t *p)
{
    if (!st || !p) return;

    memset(st, 0, sizeof(*st));
    st->p = *p;
}


detect_ied_event_t ied_process_sample(ied_state_t *st,
                               int32_t raw_sample,
                               uint64_t t_us)
{
    if (!st) return IED_NO_EVENT;

    st->sample_count++;

    float x = (float)raw_sample;

    // Bandpass Filter from 50-85 Hz
    float bp = iir_bandpass(st, x);
    //float bp = x;
	st->last_bandpass = bp;

	// Rectify the bandpassed signal
	//float rectified = fabsf(bp);

	// Calculate instantaneous power
	float power = bp * bp;

	// Apply moving average envelope smoothing
	float env = moving_average_envelope(st, power);
	st->last_envelope = env;

	// Optional: apply a 15 Hz highpass and compare raw amplitude
	float hp = iir_highpass_15hz(st, x);
	//float hp = x;
	float amp = fabsf(hp);
	st->last_amp_hp = amp;

	// Calculating the STD to base the threshold based off
	float env_std = sqrtf(fmaxf_local(st->env_var,
									  st->p.min_std * st->p.min_std));

	float amp_std = sqrtf(fmaxf_local(st->amp_var,
									  st->p.min_std * st->p.min_std));

	float env_thresh = st->env_mean + st->p.env_k * env_std;
	float amp_min_thresh = st->amp_mean + st->p.amp_min_k * amp_std;
	float amp_artifact_thresh = st->amp_mean + st->p.amp_artifact_k * amp_std;

	st->last_env_thresh = env_thresh;
	st->last_amp_min_thresh = amp_min_thresh;
	st->last_amp_artifact_thresh = amp_artifact_thresh;

	// Warmup baseline phase
	if (st->sample_count < st->p.warmup_samples) {
		update_baseline(env, st->p.baseline_alpha, &st->env_mean, &st->env_var);
		update_baseline(amp, st->p.baseline_alpha, &st->amp_mean, &st->amp_var);
		return IED_NO_EVENT;
	}

	// Refractory gate
	if (t_us < st->refractory_until_us) {
		return IED_IN_REFRACTORY;
	}

	// Candidate IED from envelope
	uint8_t envelope_crossed = (env >= env_thresh);

	if (!envelope_crossed) {
		// Only update baseline when not seeing candidate activity
		update_baseline(env, st->p.baseline_alpha, &st->env_mean, &st->env_var);
		update_baseline(amp, st->p.baseline_alpha, &st->amp_mean, &st->amp_var);
		return IED_NO_EVENT;
	}

	// Step 5: amplitude must exceed 10 SD
	if (amp < amp_min_thresh) {
		return IED_REJECTED_LOW_AMPLITUDE;
	}

	// Step 6: reject artifact if amplitude exceeds 100 SD
	if (amp > amp_artifact_thresh) {
		return IED_REJECTED_ARTIFACT;
	}

	// Step 7: valid IED
	st->refractory_until_us = t_us + st->p.refractory_us;

	return IED_DETECTED;
}
















