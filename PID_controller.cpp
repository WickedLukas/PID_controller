#include "PID_controller.h"
#include <ema_filter.h>

#include <Arduino.h>

PID_controller::PID_controller(float K_p_new, float K_i_new, float K_d_new, float max_mv_new, float max_iTerm_new,
							   float ema_filter_p_new, float ema_filter_d_new, bool derivative_on_measurement_new, float sample_time_new)
{
	K_p = K_p_new;
	K_i = K_i_new;
	K_d = K_d_new;

	max_mv = abs(max_mv_new);
	max_iTerm = abs(max_iTerm_new);

	ema_filter_p = abs(ema_filter_p_new);
	ema_filter_d = abs(ema_filter_d_new);

	derivative_on_measurement = derivative_on_measurement_new;

	sample_time = sample_time_new; // use dT as sample time if sample_time is smaller than dT

	reset();
}

void PID_controller::set_K_p(float K_p_new)
{
	K_p = K_p_new;
}

void PID_controller::set_K_i(float K_i_new)
{
	K_i = K_i_new;
}

void PID_controller::set_K_d(float K_d_new)
{
	K_d = K_d_new;
}

void PID_controller::set_max_mv(float max_mv_new)
{
	max_mv = abs(max_mv_new);
}

void PID_controller::set_max_iTerm(float max_iTerm_new)
{
	max_iTerm = abs(max_iTerm_new);
}

void PID_controller::set_ema_filter_p(float ema_filter_p_new)
{
	ema_filter_p = ema_filter_p_new;
}

void PID_controller::set_ema_filter_d(float ema_filter_p_new)
{
	ema_filter_d = ema_filter_p_new;
}

void PID_controller::set_sample_time(float sample_time_new)
{
	sample_time = sample_time_new;
}

float PID_controller::get_pTerm()
{
	return pTerm;
}

float PID_controller::get_iTerm()
{
	return iTerm;
}

float PID_controller::get_dTerm()
{
	return dTerm;
}

float PID_controller::get_mv(float sp, float pv, float dT)
{
	if (dT <= 0)
	{
		return mv;
	}

	dT_pid += dT;

	if (dT_pid >= sample_time)
	{
		error = sp - pv;

		// calculate pTerm
		if (ema_filter_p < 1)
		{
			error_filtered = ema_filter(error, error_filtered, ema_filter_p);
			pTerm = K_p * error_filtered;
		}
		else
		{
			pTerm = K_p * error;
		}

		// calculate dTerm
		if (derivative_on_measurement)
		{
			// derivative on measurement to get rid of derivative kick
			error_d = -pv;
		}
		else
		{
			error_d = error;
		}

		if (ema_filter_d < 1)
		{
			// filter error_d for derivative controller
			error_d_filtered = ema_filter(error_d, error_d_filtered, ema_filter_d);
			dTerm = K_d * (error_d_filtered - error_d_filtered_last) / dT_pid;

			error_d_filtered_last = error_d_filtered;
		}
		else
		{
			dTerm = K_d * (error_d - error_d_last) / dT_pid;

			error_d_last = error_d;
		}

		// calculate iTerm
		iTerm += K_i * dT_pid * error; // multiplying K_i inside the sum allows to adjust K_i on the fly

		// to prevent integral windup, use last iTerm if iterm, or the absolute manipulated variable, would exceed their limits
		if ((abs(iTerm) > max_iTerm) || ((mv >= max_mv) && (error > 0)) || ((mv <= -max_mv) && (error < 0)))
		{
			iTerm = iTerm_last;
		}
		iTerm_last = iTerm;

		mv = pTerm + iTerm + dTerm;

		// limit pid output
		mv = constrain(mv, -max_mv, max_mv);

		dT_pid = 0;
	}

	return mv;
}

void PID_controller::reset()
{
	dT_pid = 0;

	mv = 0;

	error_filtered = 0;
	error_d_last = 0;
	error_d_filtered = 0;
	error_d_filtered_last = 0;

	iTerm = 0;
	iTerm_last = 0;
}
