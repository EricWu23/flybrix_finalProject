/*
*  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
*
*  http://www.flybrix.com
*/

#ifndef PID_h
#define PID_h

#include <cstdint>
#include "utility/clock.h"

class IIRfilter {
public:
	IIRfilter(float _output, float _time_constant) {
		out = _output;
		tau = _time_constant;
	};
	float update(float in, float dt) {
		return out = (in * dt + out * tau) / (dt + tau);
	};

private:
	float out;
	float tau;
};

struct __attribute__((packed)) PIDSettings {
	PIDSettings() : PIDSettings(0, 0, 0, 0, 0, 0, 0) {
	}
	PIDSettings(float kp, float ki, float kd, float integral_windup_guard, float d_filter_time, float setpoint_filter_time, float command_to_value)
		: kp{ kp }, ki{ ki }, kd{ kd }, integral_windup_guard{ integral_windup_guard }, d_filter_time{ d_filter_time }, setpoint_filter_time{ setpoint_filter_time }, command_to_value{ command_to_value } {
	}

	bool verify() const {
		return integral_windup_guard >= 0.0 && d_filter_time >= 0.0 && setpoint_filter_time >= 0.0;
	}

	float kp;
	float ki;
	float kd;
	float integral_windup_guard;
	float d_filter_time;
	float setpoint_filter_time;
	float command_to_value;
};

static_assert(sizeof(PIDSettings) == 7 * 4, "Data is not packed");

class PID {
public:
	explicit PID(const PIDSettings& settings)
		: Kp{ settings.kp },
		Ki{ settings.ki },
		Kd{ settings.kd },
		integral_windup_guard{ settings.integral_windup_guard },
		d_filter{ 0.0f, settings.d_filter_time },
		setpoint_filter{ 0.0f, settings.setpoint_filter_time },
		command_to_value{ settings.command_to_value } {};

	void isWrapped(bool wrapped = true) {
		degrees = wrapped;
	}

	ClockTime lastTime() const {
		return last_time;
	}

	float pTerm() const {
		return p_term;
	}

	float iTerm() const {
		return i_term;
	}

	float dTerm() const {
		return d_term;
	}

	float input() const {
		return input_;
	}

	float setpoint() const {
		return setpoint_;
	}

	float desiredSetpoint() const {
		return desired_setpoint_;
	}

	float commandToValue() const {
		return command_to_value;
	}

	void setInput(float v) {
		input_ = v;
	}

	void setSetpoint(float v) {
		desired_setpoint_ = v;
	}

	void setFeedback(float v) {
		feedback_ = v;
	}

	void setTimer(ClockTime now) {
		last_time = now;
	}

	float Compute(ClockTime now) {
		float delta_time = (now - last_time) / 1000000.0;

		setpoint_ = setpoint_filter.update(desired_setpoint_, delta_time);

		float error = setpoint_ - input_;

		if (degrees) {
			while (error < -180.0f) {
				error += 360.0f;
			}
			while (error >= 180.0f) {
				error -= 360.0f;
			}
		}

		// Calculate p_term 
		p_term = Kp*error; // You populate this! Function of Kp and error
		// Calculate i_term
		i_term = Ki*error_integral; // You populate this! Function of Ki and error_integral
		// Update integral error (error_integral term)
		// You put code here to calculate the error_integral term!
    error_integral = error_integral+error*delta_time;
		// Perform integral windup protection
		float windup_limit = integral_windup_guard;
		if (error_integral > windup_limit) {
			error_integral = windup_limit;
		}
		else if (error_integral < -windup_limit) {
			error_integral = -windup_limit;
		}
		// Calculate r term
		r_term = Kd*feedback_;// You populate this! Function of Kd and feedback_

		previous_error = error;
		last_time = now;

		// Return the PIR output command
		float pir_output = p_term + i_term + r_term; // You populate this! Function of p_term, i_term, and r_term
		return pir_output;
	};

	void IntegralReset() {
		error_integral = 0.0f;
	};

private:
	float Kp;
	float Ki;
	float Kd;
	float integral_windup_guard;
	IIRfilter d_filter;
	IIRfilter setpoint_filter;
	float command_to_value;

	float input_{ 0.0f }, setpoint_{ 0.0f };
	float desired_setpoint_{ 0.0f };
	float feedback_{ 0.0f };

	ClockTime last_time{ ClockTime::zero() };
	float p_term{ 0.0f };
	float i_term{ 0.0f };
	float d_term{ 0.0f };
	float r_term{ 0.0f };

	bool degrees{ false };  // unwrap error terms for angle control in degrees

	float previous_error{ 0.0f };
	float error_integral{ 0.0f };
};

#endif
