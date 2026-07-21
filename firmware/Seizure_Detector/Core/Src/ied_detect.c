/*
 * ied_detect.c
 *
 *  Created on: May 28, 2026
 *      Author: vijit
 */

#include "ied_detect.h"
#include <math.h>
#include <string.h>

#define BP_SOS_SECTIONS 2

static inline float fmaxf_local(float a, float b){
    return (a > b) ? a : b;
}

static float iir_bandpass_50_85hz(ied_state_t *st, float x){
    /*
     * 4th-order Butterworth bandpass implemented as 2 SOS biquads.
     * Designed for:
     * fs = 5000 Hz
     * passband = 50–85 Hz
     *
     * scipy:
     * sos = scipy.signal.butter(2, [50, 85], btype='bandpass',
     *                           fs=5000, output='sos')
     *
     * Each row:
     * b0, b1, b2, a0, a1, a2
     *
     * a0 is 1.0, so the difference equation is:
     * y[n] = b0*x[n] + z1
     * z1 = b1*x[n] - a1*y[n] + z2
     * z2 = b2*x[n] - a2*y[n]
     */

	static const float sos[BP_SOS_SECTIONS][6] = {
		{
			4.6895284449e-04f,
			9.3790568899e-04f,
			4.6895284449e-04f,
			1.0000000000e+00f,
		   -1.9540201726e+00f,
			9.6368791748e-01f
		},
		{
			1.0000000000e+00f,
		   -2.0000000000e+00f,
			1.0000000000e+00f,
			1.0000000000e+00f,
		   -1.9705898680e+00f,
			9.7510260125e-01f
		}
	};

	float section_input = x;

	for (uint32_t i = 0; i < BP_SOS_SECTIONS; i++) {
		const float b0 = sos[i][0];
		const float b1 = sos[i][1];
		const float b2 = sos[i][2];
		const float a1 = sos[i][4];
		const float a2 = sos[i][5];

		const float section_output = b0 * section_input + st->bp_z1[i];

		st->bp_z1[i] = b1 * section_input - a1 * section_output + st->bp_z2[i];
		st->bp_z2[i] = b2 * section_input - a2 * section_output;

		section_input = section_output;
	}

	return section_input;
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
    float bp = iir_bandpass_50_85hz(st, x);

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

	// Verify the candidate is a real IED
	if (st->candidate_active) {
	    st->candidate_count++;

	    if (amp > st->candidate_max_amp) {
	        st->candidate_max_amp = amp;
	    }

	    if (fabsf(x) > st->candidate_max_raw) {
	        st->candidate_max_raw = fabsf(x);
	    }

	    if (env > st->candidate_max_env) {
	        st->candidate_max_env = env;
	    }

	    if (st->candidate_count < st->p.candidate_window_samples) {
	        return IED_NO_EVENT;
	    }

	    /* Candidate window complete: now decide */
	    st->candidate_active = 0;

	    if (st->candidate_max_raw > st->p.raw_abs_artifact_limit) {
	        return IED_REJECTED_ARTIFACT;
	    }

	    if (st->candidate_max_amp > amp_artifact_thresh) {
	        return IED_REJECTED_ARTIFACT;
	    }

	    if (st->candidate_max_amp < amp_min_thresh) {
	        return IED_REJECTED_LOW_AMPLITUDE;
	    }

	    st->refractory_until_us = st->candidate_time_us + st->p.refractory_us;
	    return IED_DETECTED;
	}

	/* If no active candidate and envelope crosses, start candidate window */
	if (envelope_crossed) {
	    st->candidate_active = 1;
	    st->candidate_count = 0;
	    st->candidate_max_amp = amp;
	    st->candidate_max_raw = fabsf(x);
	    st->candidate_max_env = env;
	    st->candidate_time_us = t_us;

	    return IED_NO_EVENT;
	}

	if (!envelope_crossed) {
		// Only update baseline when not seeing candidate activity
		update_baseline(env, st->p.baseline_alpha, &st->env_mean, &st->env_var);
		update_baseline(amp, st->p.baseline_alpha, &st->amp_mean, &st->amp_var);
		return IED_NO_EVENT;
	}

	/* Old Single Candidate Event System
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
	*/
}
















