/*
 * FPID.h
 *
 *  Created jan 2022
 *      Author: Tijs
 */

#ifndef __FPID_H_
#define __FPID_H_

#include <stdlib.h>
#include <math.h>

class FPID
{
    public:
        typedef struct
        {
            double kF;
            double kP;
            double kI;
            double kD;
            double setpoint;
            double outputFilter;
        } fpid_settings_t;

        FPID(fpid_settings_t* s, const double* input, double* output);

        // Aligns the output of the controller with the current output
        // Call this before (re)starting the PID loop to prevent a large start-up kick
	    void alignOutput();
        bool calculate(const double dt = 1.0);

        // Primary interface, with external pointers
        void setInputPtr(const double* input) { _input_ptr = input; };
        void setOuputPtr(double* output) { _output_ptr = output; };

        // Minimum and Maximum output values
	    void setOutputLimits(const double);
	    void setOutputLimits(const double, const double);
	    void getOutputLimits(double*, double*);
	    
        // Set the maximum rate the output can increase per cycle.
	    void setOutputRampRate(const double);

        /**Set a filter on the output to reduce sharp oscillations.
         * 0.1 is likely a sane starting value. 
         * Larger values P and D oscillations, but force larger I values.
         * Uses an exponential rolling sum filter, according to a simple
         * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre>
         * output valid between [0..1), meaning [current output only.. historical output only)
         */
	    void setOutputFilter(const double);

        /**Set the maximum output value contributed by the I component of the system
         * this can be used to prevent large windup issues and make tuning simpler
         * @param maximum. Units are the same as the expected output value
         */
    	void setMaxIOutput(const double);

        // Ramp the setpoint no further than this from the input
        /** Set a limit on how far the setpoint can be from the current input value
         *  Can simplify tuning by helping tuning over a small range applied to a much larger range.
         *  this limits the reactivity of P term, and restricts impact of large D term
         *  during large setpoint adjustments. Increases lag and I term if range is too small.
         */
	    void setSetpointRange(const double);
    
    protected:
        // Initialize the settings struct, resets values. Don't call if settings come from NVS
        // void init();
	    void set_output(double);

        // pointers to the outside world
        fpid_settings_t *_settings_ptr;
        const double *_input_ptr;
        double *_output_ptr;

        double _minOutput = -INFINITY;
        double _maxOutput = INFINITY;
        double _outputRampRate = INFINITY;

        double _setpointRange = INFINITY;   // Can deviate this much from 'input'
        double _maxIOutput = INFINITY;

        // Forward term model, o
        double forwardTerm();

        // Internal state variables
        double _errorsum;
    	double _prv_input;
        double _prv_output;
    	
    	int _outputClampedByRamprate = 0;
	    int _outputClampedByMinMax = 0;
};

class FPIDWrapped : public FPID
{
    public:
        FPIDWrapped() : FPID(&_settings_inst, &_input_inst, &_output_inst) {};

        // Primary interface, internal values
        void setInput(const double input) { _input_inst = input; };
        void setSetpoint(const double setpoint) { _settings_inst.setpoint = setpoint; };
        double getSetpoint() { return _settings_inst.setpoint; };

        void setParameters(const double p, const double i, const double d) { setParameters(p, i, d, _settings_inst.kF); };
        void setParameters(const double p, const double i, const double d, const double f)
        { 
            _settings_inst.kP = p; 
            _settings_inst.kI = i; 
            _settings_inst.kD = d; 
            _settings_inst.kF = f; 
        };

    private:
        FPID::fpid_settings_t _settings_inst;
        double _input_inst;
        double _output_inst;
};

#endif /* __MINIPID_H_ */
