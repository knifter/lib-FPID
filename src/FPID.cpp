/*
 * FPID.cpp
 *
 *  Created jan 2022
 *      Author: Tijs
 */

#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include "FPID.h"

#ifdef DEBUG_FPID
	#include "tools-log.h"
#endif

// Defaults
#ifndef FPID_TBH_HYSTERESIS
    #define FPID_TBH_HYSTERESIS     0.01
#endif

int clamp(double *value, const double min, const double max);
bool isbetween(const double value, const double min, const double max);

FPID::FPID(fpid_settings_t *s, const double *input, double *output) :
    _settings_ptr(s), _input_ptr(input), _output_ptr(output)
{
};

// void FPID::init()
// {
// 	_settings->kP = 1.0;
// 	_settings->kI = 0.0;
// 	_settings->kD = 0.0;
// 	_settings->kF = 0.0;
// 	_settings->setpoint = 0;
// 	_settings->outputFilter = 0.1;

//     _maxIOutput=0;
// 	_errorsum = 0.0;
// 	_maxOutput=0;
// 	_minOutput=0;
//     _prv_input = NAN; // marks first run
//     _prv_output = NAN;

// 	_outputRampRate = 0;
// 	_setpointRate = NAN;
// };

/**
 * Aligns the internal state with the current output
 */
void FPID::alignOutput()
{
    _prv_input = NAN;
    _prv_output = NAN;
#ifdef FPID_TAKEBACKHALF
	_tbh_prv_errorsum = NAN;
#endif

	clamp(_output_ptr, _minOutput, _maxOutput);

	// Reset the errorsum, will be recovered in calculate() from _output
	_errorsum = NAN;
};

/**Set the maximum output value contributed by the I component of the system
 * this be used to prevent large windup issues and make tuning simpler
 * @param maximum. Units are the same as the expected output value
 */
void FPID::setMaxIOutput(const double maximum)
{
	/* Internally maxError and Izone are similar, but scaled for different purposes.
	 * The maxError is generated for simplifying math, since calculations against
	 * the max error are far more common than changing the I term or Izone.
	 */
	_maxIOutput=maximum;
}

/**Specify a maximum output. If a single parameter is specified, the minimum is
 * set to (-maximum).
 * @param output
 */
void FPID::setOutputLimits(const double output)
{ 
	setOutputLimits(-output, output);
};

/**
 * Specify a maximum output.
 * @param minimum possible output value
 * @param maximum possible output value
 */
void FPID::setOutputLimits(const double minimum, const double maximum)
{
	if(maximum>minimum)
    {
    	_maxOutput=maximum;
	    _minOutput=minimum;
    }else{
    	_maxOutput=minimum;
	    _minOutput=maximum;
    };
		
	// Ensure the bounds of the I term are within the bounds of the allowable output swing
	if(_maxIOutput == 0 || _maxIOutput>(maximum-minimum) )
	{
		setMaxIOutput(_maxOutput-_minOutput);
	};
};

/** Calculate the PID value needed to hit the target setpoint.
* Automatically re-calculates the output at each call.
* @param dt Time differential between calls
* @return true if in-loop and not wound-up (!integral frozen)
*/
bool FPID::calculate(const double dt)
{
	if(!(dt > 0.0) || !isfinite(dt))
	{
#ifdef DEBUG_FPID
		WARNING(" dt = %f", dt);
#endif
        *_output_ptr = NAN;
		return false;
	};

    // Sample settings for this loop
	double setpoint = _settings_ptr->setpoint;
    double input = *_input_ptr;

	if(!isfinite(input))
	{
#ifdef DEBUG_FPID
		ERROR("input: NAN");
#endif
        *_output_ptr = NAN;
		return false;
	};

	//Ramp the setpoint used for calculations if user has opted to do so
    clamp(&setpoint, input - _setpointRange, input + _setpointRange);

	//Do the simple parts of the calculations
	double error = setpoint - input;

#ifdef FPID_ROTATIONAL
	// Rotational correction, error can never be bigger than +-180
	//  and we also need to move to the shortes rotation side
	if(error > 180)
		error -= 360;
	if(error < -180)
		error += 360;
#endif

	/*********** FORWARD TERMS ***********************************************************************************/
	//Calculate F output. Notice, this depends only on the setpoint, and not the error.
#ifdef FPID_FORWARD_LINEAR
    double Foutput_linear = _settings_ptr->kF * (_settings_ptr->setpoint- _settings_ptr->kF_offset);
#endif // FPID_FORWARD_LINEAR

#ifdef FPID_FORWARD_DSETPOINT
	//Calculate Derivative-on-setpoint (which forward term as it depens solely on SP)

	if(!isfinite(_prv_setpoint))
		_prv_setpoint = setpoint;

	double Foutput_dsetpoint = _settings_ptr->kDsetpoint*(setpoint - _prv_setpoint);
	_prv_setpoint = setpoint;
#endif

	/*********** PROPORTIONAL TERMS ***********************************************************************************/
	double Poutput = _settings_ptr->kP * error;

#ifdef FPID_PROOT
	double PRoutput = _settings_ptr->kPR * sqrt(error);
#endif

	/*********** DERIVATIVE TERMS ***********************************************************************************/
    // First run/sync
	if(!isnan(_prv_input))
		_prv_input = input;
		
	//Calculate Derrivative-on-measurement
	//Note, this is negative. this actually "slows" the system if it's doing
	//the correct thing, and small values helps prevent output spikes and overshoot
	// D-term filter can be applied as well if PID loop runs fast and input changes (discretely) slow
	double dterm = -1 * (input - _prv_input) / dt;

	if(isnan(_prv_dterm))
		_prv_dterm = dterm;

#ifdef FPID_D_FILTER
	dterm = dterm*(1 - _settings_ptr->D_filter) + _prv_dterm*_settings_ptr->D_filter;
#endif
	double Doutput = _settings_ptr->kD * dterm;

#ifdef FPID_DOUBLE_D
	double ddterm = (dterm - _prv_dterm) / dt;
	double DDoutput = _settings_ptr->kDD * ddterm;
#endif

	_prv_dterm = dterm;
	_prv_input = input;

	/*********** INTEGRAL TERMS ***********************************************************************************/
	//The Iterm is more complex. There's several things to factor in to make it easier to deal with.
	// 1. The multiplication with I-gain is done when adding to the sum, this prevents the bump on I-gain changes
	// 2. prevent further windup by not increasing errorSum if output=maxOutput or output=minOutput
	// 3. prevent windup by not increasing errorSum if we're already running against our max Ioutput
	// 3b. But only if the outputclamp and error have the same sign (direction)
	bool freeze_integral = (_outputClampedByMinMax && _outputClampedByMinMax*error > 0);
#ifdef FPID_OUTPUT_RAMPRATE
	if(_outputClampedByRamprate)
		freeze_integral = true;
#endif

#ifdef DEBUG_FPID
	DBG("freeze = %d, ramp(%d), minmax(%d) (%f)", freeze_integral,
	#ifdef FPID_OUTPUT_RAMPRATE
			_outputClampedByRamprate,
	#else
			0,
	#endif
		 _outputClampedByMinMax, _outputClampedByMinMax*error);
#endif	

	// If errorsum is NAN we need to align it with the current _output not cause a big jump
	if(isnan(_errorsum))
	{
		_errorsum = 0;
		// If there is a previous output, it must be the result of the 
		// intergral term + forward terms, we can run that backwards
		// and recover errorsum, otherwise we start with 0 integral now
		if(isfinite(*_output_ptr))
		{
			_errorsum = *_output_ptr;
	#ifdef FPID_FORWARD_LINEAR
			_errorsum -= Foutput_linear;
	#endif
		// FPID_FORWARD_DSETPOINT is not needed as it is a D-term and can just be added this cycle
		};
	};

	// If all good, increase integral
	if(!freeze_integral)
		_errorsum += _settings_ptr->kI * error * dt;

#ifdef FPID_TAKEBACKHALF
	if(_settings_ptr->takebackhalf)
	{
		// Take-Back-Half
		double error_hyst = setpoint*FPID_TBH_HYSTERESIS;
		// first loop detection
		if(isnan(_tbh_prv_errorsum))
		{
			if(error>0)
			{
				// If error positive, output will rise, integral will grow
				_tbh_limit = +error_hyst;
				_tbh_prv_errorsum = 0;
			}else{
				// If error positive, output will fall, integral will shrink
				_tbh_limit = -error_hyst;
				_tbh_prv_errorsum = _errorsum;
			};
	#ifdef DEBUG_FPID
			DBG("TBH: init, limit = %.02f, sum_prv = %.02f", _tbh_limit, _tbh_prv_errorsum);
	#endif
		};
		// if zero crossing (with hysteresis), average the intergral between current value 
		// and value of last crossing: Take Back Half
		if(_tbh_limit > 0 && error > _tbh_limit)
		{
			DBG("TBH: crossing from - to +, prv_I = %f", _tbh_prv_errorsum);
			_tbh_limit = -error_hyst;
			_errorsum = _tbh_prv_errorsum = (_errorsum + _tbh_prv_errorsum)/2;
		};
		if(_tbh_limit < 0 && error < _tbh_limit)
		{
			DBG("TBH: crossing from + to -, prv_I = %f", _tbh_prv_errorsum);
			_tbh_limit = +error_hyst;
			_errorsum = _tbh_prv_errorsum = (_errorsum + _tbh_prv_errorsum)/2;
		};
	}; // if(take_back_half)
#endif // FPID_TAKEBACKHALF

	// 3. maxIoutput restricts the amount of output contributed by the Iterm.
    clamp(&_errorsum, -1*_maxIOutput, _maxIOutput);

	// Now our I output term is just the sum as the I factor is already processed while adding to the sum previously
	double Ioutput = (_settings_ptr->kI > 0.0) ? _errorsum : 0.0;

	//And, finally, we can just add the terms up
	double output = Poutput + Ioutput + Doutput;
#ifdef FPID_FORWARD_LINEAR
	output += Foutput_linear;
#endif
#ifdef FPID_FORWARD_DSETPOINT
	output += Foutput_dsetpoint;
#endif
#ifdef FPID_PROOT
	output += PRoutput;
#endif
#ifdef FPID_DOUBLE_D
	output += DDoutput;
#endif

#ifdef DEBUG_FPID
	DBG("in:%f error = %.2f FPID = F:%f + P:%f + I:%f%s + D:%f = %f", 
	*_input_ptr,
	error, 0
	#ifdef FPID_FORWARD_LINEAR
		+ Foutput_linear
	#endif
	#ifdef FPID_FORWARD_DSETPOINT
		+ Foutput_dsetpoint,
	#endif
		, Poutput
	#ifdef FPID_PROOT
		+ PRoutput
	#endif
		, Ioutput, freeze_integral ? "(frozen)":"", Doutput, output);
#endif // DEBUG_FPID

#ifdef FPID_OUTPUT_RAMPRATE
    // First run/sync:
	// WARNING: if ramprate limited and isnan(_output_ptr), we'll keep outputting NaN, as that makes sense if ramprate is limited to prevent a big jump at turn-on.
    if(isnan(_prv_output))
        _prv_output = *_output_ptr;

	// Limit the output by ramprate
	_outputClampedByRamprate = clamp(&output, 
        _prv_output - _outputRampRate, 
        _prv_output + _outputRampRate);
#else
    // First run/sync, if no ramprate limit we can just set the output to the first value
    if(!isfinite(_prv_output))
		_prv_output = output;
#endif // FPID_OUTPUT_RAMPRATE

	// Limit the output by min/maxOutput
	_outputClampedByMinMax = clamp(&output, _minOutput, _maxOutput);

	// Filter the Output: 1 = old output (low-pass), 0 = new output, -1 = subtract old output (high-pass, not implemented yet)
#ifdef FPID_OUTPUT_FILTER
	output = output*(1 - _settings_ptr->output_filter) + _prv_output*_settings_ptr->output_filter;
#endif
	_prv_output = output;

    *_output_ptr = output;
#ifdef DEBUG_FPID
	// DBG("output = %f -> 0x%p = %f", output, _output_ptr, *_output_ptr);
#endif
	return !freeze_integral;
};

/**
 * Forces a value into a specific range
 * @param value input value
 * @param min maximum returned value
 * @param max minimum value in range
 * @return 0 if it's within provided range, -1 (min) or +1 (max) otherwise
 */
int clamp(double* value, const double min, const double max)
{
	if(*value > max)
	{
		*value = max;
		return 1;
	};
	if(*value < min)
	{
		*value = min;
		return -1;
	};
	return 0;
};

/**
 * Test if the value is within the min and max, inclusive
 * @param value to test
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return
 */
bool isbetween(const double value, const double min, const double max)
{
		return (min<value) && (value<max);
};

