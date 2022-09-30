#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

class PID_controller
{

public:
	PID_controller(float K_p_new, float K_i_new, float K_d_new, float max_mv_new, float max_iTerm_new,
				   float ema_filter_p_new = 1, float ema_filter_d_new = 1, bool derivative_on_measurement_new = false, float sample_time_new = 0);

	void set_K_p(float K_p_new); // set proportional gain
	void set_K_i(float K_i_new); // set integral gain
	void set_K_d(float K_d_new); // set derivative gain

	void set_max_mv(float max_mv_new);		 // set absolute maximum for manipulated variable
	void set_max_iTerm(float max_iTerm_new); // set absolute maximum for integral term

	void set_ema_filter_p(float ema_filter_p_new); // set ema-value for proportional error filter
	void set_ema_filter_d(float ema_filter_d_new); // set ema-value for derivative error filter

	void set_sample_time(float sample_time_new); // set targeted pid-sample time in seconds

	float get_pTerm(); // get proportional term
	float get_iTerm(); // get integral term
	float get_dTerm(); // get derivative

	// calculate manipulated variable (mv) from setpoint (sp) and process variable (pv) using the time difference (dT) in seconds
	float get_mv(float sp_new, float pv_new, float dT);

	void reset(); // reset PID controller

private:
	float K_p; // proportional gain
	float K_i; // integral gain
	float K_d; // derivative gain

	float max_mv;	 // absolute maximum for manipulated variable (mv)
	float max_iTerm; // absolute maximum for integral term (iTerm)

	float ema_filter_p; // ema filter value for proportional error
	float ema_filter_d; // ema filter value for derivative error

	bool derivative_on_measurement; // calculate dTerm using derivative on measurement

	float sample_time; // targeted pid sample time (same unit as dT!)
	float dT_pid;	   // current pid time

	float mv; // manipulated variable (mv)

	float error;								   // error
	float error_filtered;						   // filtered error
	float error_d, error_d_last;				   // error used for derivative term
	float error_d_filtered, error_d_filtered_last; // filtered error used for derivative term

	float pTerm;			 // proportional term
	float iTerm, iTerm_last; // integral term
	float dTerm;			 // derivative term
};

#endif
