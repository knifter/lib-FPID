/*
 * MiniPID.h
 *
 *  Created on: 5 apr. 2018
 *      Author: Tijs
 */

#ifndef __MINIPID_H_
#define __MINIPID_H_

#include <stdlib.h>
#include <math.h>

class FPID
{
    public:
        typedef struct
        {
            double kP;
            double kI;
            double kD;
            double kF;
            double setpoint;
            double outputFilter;
            bool active;
        } fpid_settings_t;

    	FPID();
        FPID(fpid_settings_t* s, double* input, double* output);

        // Aligns the output of the controller with the current output
        // Call this before (re)starting the PID loop to prevent a large start-up kick
	    void alignOutput();
        bool calculate(double dt = 1.0);

        // Primary interface, with external pointers
        void setInputPtr(double* input);
        void setOuputPtr(double* output);

        // Primary interface, internal values
        void setInput(double);
        void setSetpoint(double);
        double getSetpoint();

       	void setParameters(const double p, const double i, const double d);
    	void setParameters(const double p, const double i, const double d, const double f);

        // Minimum and Maximum output values
	    void setOutputLimits(const double);
	    void setOutputLimits(const double, const double);
	    void getOutputLimits(double*, double*);
	    
    

        // Set the maximum rate the output can increase per cycle.
	    void setOutputRampRate(double);

        /**Set a filter on the output to reduce sharp oscillations.
         * 0.1 is likely a sane starting value. 
         * Larger values P and D oscillations, but force larger I values.
         * Uses an exponential rolling sum filter, according to a simple
         * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre>
         * output valid between [0..1), meaning [current output only.. historical output only)
         */
	    void setOutputFilter(double);

        /**Set the maximum output value contributed by the I component of the system
         * this can be used to prevent large windup issues and make tuning simpler
         * @param maximum. Units are the same as the expected output value
         */
    	void setMaxIOutput(const double);

        // Ramp the setpoint no further than this from the input
        /** Set a limit on how far the setpoint can be from the current position
         *  Can simplify tuning by helping tuning over a small range applies to a much larger range.
         *  this->limits the reactivity of P term, and restricts impact of large D term
         *  during large setpoint adjustments. Increases lag and I term if range is too small.
         */
	    void setSetpointRate(double);
    
    protected:
        // Initialize the settings struct, resets values. Don't call if settings come from NVS
        void init();
	    void set_output(double);

        // pointers to the outside world
        fpid_settings_t *_settings = nullptr;
        double* _input;
        double* _output;
        // If no input/output ptr is set, these are used:
        double input_val = 0;
        double output_val = 0;

        double _minOutput = -INFINITY;
        double _maxOutput = INFINITY;
        double _outputRampRate = INFINITY;

        double _setpointRate = INFINITY;
        double _maxIOutput = INFINITY;

        // Forward term model, o
        double forwardTerm();

        // Internal state variables
        double _errorsum;
    	double _prv_input;
        double _prv_output;
    	
    	bool _outputClampedByRamprate = 0;
	    bool _outputClampedByMinMax = 0;
};

#endif /* __MINIPID_H_ */
