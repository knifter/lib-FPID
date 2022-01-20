/*
 * MiniPID.cpp
 *
 *  Created on: 5 apr. 2018
 *      Author: Tijs (not original author)
 */

/**
* Small, easy to use PID implementation with advanced controller capability.<br>
* Minimal usage:<br>
* setPID(p,i,d); <br>
* ...looping code...{ <br>
* output=getOutput(sensorvalue,target); <br>
* }
*
* @see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/improving-the-beginners-pid-introduction
*/
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include "FPID.h"

int clamp(double* value, double min, double max);
bool isbetween(double value, double min, double max);

//**********************************
//Constructor functions
//**********************************
FPID::FPID()
{
    _settings = static_cast<fpid_settings_t*>(malloc(sizeof(fpid_settings_t)));
    _input = &input_val;
    _output = &output_val;
    init();
};

FPID::FPID(fpid_settings_t* s, double* input, double* output) :
    _settings(s), _input(input), _output(output)
{
};

void FPID::init()
{
	_settings->kP = 1.0;
	_settings->kI = 0.0;
	_settings->kD = 0.0;
	_settings->kF = 0.0;
	_settings->setpoint = 0;
	_settings->outputFilter = 0.1;

    _maxIOutput=0;
	_errorsum = 0.0;
	_maxOutput=0;
	_minOutput=0;
    _prv_input = NAN; // marks first run
    _prv_output = NAN;

	_outputRampRate = 0;
	_setpointRate = NAN;
};

/**
 * Aligns the internal state with the current output
 */
void FPID::alignOutput()
{
    _prv_input = NAN;
    _prv_output = NAN;

    // calculate (estimate) the required integral sum to obtain current output
    // i.e.: take over current output
	_errorsum = *_output - forwardTerm();
};

//**********************************
//Configuration functions
//**********************************
/**
 * Configure the Proportional gain parameter. <br>
 * this->responds quicly to changes in setpoint, and provides most of the initial driving force
 * to make corrections. <br>
 * Some systems can be used with only a P gain, and many can be operated with only PI.<br>
 * For position based controllers, this->is the first parameter to tune, with I second. <br>
 * For rate controlled systems, this->is often the second after F.
 *
 * @param p Proportional gain. Affects output according to <b>output+=P*(setpoint-current_value)</b>
 */
void FPID::setParameters(const double p, const double i, const double d)
{
	_settings->kP = p;
	_settings->kI = i;
	_settings->kD = d;
};

void FPID::setParameters(const double p, const double i, const double d, const double f)
{
	_settings->kP = p;
	_settings->kI = i;
	_settings->kD = d;
	_settings->kF = f;
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

//**********************************
//Primary operating functions
//**********************************

/**Set the target for the PID calculations
 * @param setpoint
 */
void FPID::setSetpoint(double setpoint)
{
	_settings->setpoint = setpoint;
};

double FPID::getSetpoint()
{
	return _settings->setpoint;
};

/** Calculate the PID value needed to hit the target setpoint.
* Automatically re-calculates the output at each call.
* @param dt Time differential between calls
* @return whether the output is clamped or the intergral has wound up: status
*/
bool FPID::calculate(double dt)
{
    // Sample settings for this loop
	double sp = _settings->setpoint;
    double input = *_input;

	//Ramp the setpoint used for calculations if user has opted to do so
    sp = clamp(&sp, input - _setpointRate, input + _setpointRate);

	//Do the simple parts of the calculations
	double error = sp - input;

	//Calculate F output. Notice, this depends only on the setpoint, and not the error.
	double Foutput = forwardTerm();

	//Calculate P term
	double Poutput = _settings->kP * error;

	//Calculate D Term, derrivative on measurement
	//Note, this is negative. this actually "slows" the system if it's doing
	//the correct thing, and small values helps prevent output spikes and overshoot
    if(_prv_input == NAN)
        _prv_input = input;
	double Doutput = _settings->kD * (input - _prv_input) / dt;
	_prv_input = input;

	//The Iterm is more complex. There's several things to factor in to make it easier to deal with.
	// 1. The multiplication with I-gain is done when adding to the sum, this prevents the bump on I-gain changes
	// 2. prevent further windup by not increasing errorSum if output=maxOutput or output=minOutput
	// 3. prevent windup by not increasing errorSum if we're already running against our max Ioutput
	// 3b. But only if the outputclamp and error have the same sign (direction)
	bool freeze_integral = _outputClampedByRamprate || (_outputClampedByMinMax && _outputClampedByMinMax*error > 0);

	// If all good, increase integral
	if(!freeze_integral)
		_errorsum += _settings->kI * error * dt;

	// 3. maxIoutput restricts the amount of output contributed by the Iterm.
    clamp(&_errorsum, -1*_maxIOutput, _maxIOutput);

	// Now our I output term is just the sum as the I factor is already processed while adding to the sum previously
	double Ioutput = _errorsum;

    // First run?
    if(_prv_output == NAN)
        _prv_output = *_output;

	//And, finally, we can just add the terms up
	double output = Foutput + Poutput + Ioutput + Doutput;

	// Limit the output by ramprate
	_outputClampedByRamprate = clamp(&output, 
        _prv_output - _outputRampRate, 
        _prv_output + _outputRampRate);
	
	// Limit the output by min/maxOutput
	_outputClampedByMinMax = clamp(&output, _minOutput, _maxOutput);

	// Filter the Output
    //FIXME: NAN
	output = _prv_output * _settings->outputFilter + output*(1-_settings->outputFilter);
	_prv_output = output;

    *_output = output;
	return freeze_integral;
};


double FPID::forwardTerm()
{
    return _settings->kF * _settings->setpoint;
};

/**
 * Forces a value into a specific range
 * @param value input value
 * @param min maximum returned value
 * @param max minimum value in range
 * @return 0 if it's within provided range, -1 (min) or +1 (max) otherwise
 */
int clamp(double* value, double min, double max)
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
bool isbetween(double value, double min, double max)
{
		return (min<value) && (value<max);
};
