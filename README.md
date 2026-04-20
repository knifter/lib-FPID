# F-PID
My implementation of a simple PID loop with code and ideas combined from:
 - [MiniPID by tekdemo](https://github.com/tekdemo/MiniPID)
 - [the Arduino-PID-library from br3tbb](https://github.com/br3ttb/Arduino-PID-Library)
 - [brettbeauregards's blog](http://brettbeauregard.com/blog/)
 - [VEX Robotics Wiki](https://www.vexwiki.org/programming/controls_algorithms)
 
and the following improvements:
- [x] Feed-Forward term, base class uses a simple kF*(SetPoint - F-offset)
- [x] Derivative on measurement (derivative on setpoint is a forward term: dsetpoint/dt)
- [x] external variables: allows to store the settings_t struct in a global struct which is stored in NVS. As most applications will have an interface to edit these..
- [ ] Take-back-half
- [x] Root-P
- [x] Double D
- [x] Derivative on setpoint (is a forward term)

## Improvements wishlist
- [x] Proportional on Measurement
- [ ] Forward term learning: By storing the output for a certain amount of setpoints an estimate (interpolation) can be made for a forward term.
- [ ] Declining D on big errors
- [ ] Some kind of AutoTuning

## Design Goals
- Combining all the features and improvements on software PID loops which I find usefull
- Good integration with the application: Probably a (G)UI and some NVS
- Flexible and configurable for most cases

## Features
##### PID Functions
True to the name, the main purpose of this class is PID control.

##### Feed Forward rate setting. 
Provides an "Expected output", helpful when doing velocity control systems. 

##### Setpoint Range
Force the PID system to cap the setpoint to a range near the current input values. This allows you to tune the PID in a smaller range, and have it perform sanely during large setpoint changes. 

##### Output ramping
Allows a maximum rate of change on the output, preventing jumps in output during setpoint changes

##### Output Range
Adjustable min and maximum range, so the output can directly drive a variety of systems. 

##### Output Filtering
Helps smooth the output, preventing high-frequency oscillations.

##### I term restriction
Allows you to specify a maximum output value the I term will generate. This allows active error correction, with minimal risk of I-term windup. 

##### I term saturation detection
The I term will no longer increase when the output is already at its maximum, the output ramprate is currently limiting the output or when 
The I term and summed error will never increase if the system is already doing everything permitted to correct the system error. 

##### Simple API. 
No need for lots of convoluted calculation functions, or asyncronous calculation modes. After configuration, `getOutput()` is probably the only function you need. 

## Usage

The controller works with external pointers for input and output, and a `fpid_settings_t` struct for all tuning parameters. This makes it easy to store settings in NVS and edit them via a UI.

```cpp
#include <FPID.h>

FPID::fpid_settings_t pid_settings = {
    .kP = 1.0,
    .kI = 0.1,
    .kD = 0.05,
    .setpoint = 50.0,
    .takebackhalf = false,
};

double sensor_value = 0.0;
double output_value = 0.0;

FPID pid(&pid_settings, &sensor_value, &output_value);

void setup() {
    pid.setOutputLimits(0.0, 100.0);
    sensor_value = readSensor();
    pid.alignOutput();   // align internal state before first calculate()
}

void loop() {
    sensor_value = readSensor();
    double dt = 0.1;           // seconds since last call
    pid.calculate(dt);
    applyOutput(output_value);
}
```

> PID systems work best when called at a constant interval. This library does not handle timing — pass the actual elapsed time as `dt`.

### Getting and setting outputs
The main call is `pid.calculate(dt)`, which runs one iteration using the current `*input` and `settings.setpoint`. It returns `false` when the integral is frozen (output saturated), useful for monitoring windup state.

Update `settings.setpoint` at any time between calls — the next `calculate()` picks it up automatically.

If your output is disabled or driven by something else, call `pid.alignOutput()` before restarting the loop. This aligns the internal errorsum and previous-output state with the current output value, preventing a large startup kick.

### Configuring the PID. 
The most complex part of PID systems is the configuration. Tuning a PID process properly typically requires either significant calculation, significant trial and error, or both. 

This library is designed to produce "decent" PID results with minimal effort and time investment, by providing more extensive configuration options than most controllers.

Note, PID systems work best when the calculations are performed at constant time intervals. This PID implimentation does not handle this, and assumes the primary loop or framework handles the precise timing details.

#### `FPID(fpid_settings_t* settings, const double* input, double* output)`

Takes pointers to the settings struct, input variable, and output variable. All tuning parameters live in `fpid_settings_t` — including `kP`, `kI`, `kD`, `setpoint`, and optional terms depending on enabled features (see below). Gains can be changed at any time by writing to the struct; the next `calculate()` call will use the new values.

Tuning PID systems is out of scope of this readme, but a good examples can be found all over the internet.

#### Feed-Forward (`kF`, requires `#define FPID_FORWARD_LINEAR`)
Feed-Forward is a 4th system variable that is very helpful on systems with a target velocity, or other systems where an on-target system results in continous motion. Feed forward is not helpful on positional control systems, or other systems where being on target results in halted (or small cyclic) motion.

Conceptually, Feed-forward defines a "best guess" as to what the system output should be for a given setpoint value. Feed forward does not consider what the system is _actually_ doing, and a system driven solely by feed-forward is actually an open-loop system. Mathematically, a feed-forward only system is equivilent to `output=setpoint*F`. 

In most cases, the F term can be calculated very simply by running an output at full speed, and measuring your sensor rate. The F term is then `max_output_value/max_sensor_rate`.

For this class of systems, it's helpful to consider the F term the primary variable, and configured first. Using F in this way will result in a shorter time-to-target since you don't wait for error buildup (to aquire the I term). It's also simpler to tune and more stable since you don't have large P and D terms which will cause oscillation. The P, I, and D  values then will then add minor corrections, operating on a much smaller system error. In this setup, P and I will generally correct for non-linearities in the system such as such as drag, inertia, and friction. D is helpful for providing recovery on sudden loading of the system, or quickly switching to a new setpoint.

#### `setOutputLimits(double minimum,double maximum)`
#### `setOutputLimits(double output)`
Optional, but highly recommended to set. The set the output limits, and ensure the controller behaves when reaching the maximum output capabilities of your physical system. 

#### `setMaxIOutput(double maximum)`
Sets the maximum output generated by the I term inside the controller. This is independent of the `setOutputLimits` values. This can assist in reducing windup over large setpoint changes or stall conditions. 

#### `alignOutput()`
Aligns the internal state (errorsum, previous input/output) with the current output value. Call this before (re)starting the control loop when the output may have been driven externally — for example after manual control or after the loop was disabled. Prevents a large startup kick.

#### Output ramp rate (`FPID_OUTPUT_RAMPRATE`)
When this feature is enabled at compile time, the output is limited to a maximum rate of change per cycle. This is particularly useful for adding "inertia" to the system, preventing jerks during setpoint changes. The ramp rate is set via the settings struct.

#### Output filter (`FPID_OUTPUT_FILTER`)
When this feature is enabled at compile time, the output is low-pass filtered using an exponential rolling sum (`output_filter` in the settings struct, range [0..1)). This prevents sharp changes in the output, adding inertia and minimizing the effect of high-frequency oscillations. This adds significant stability to systems with poor tunings, but at the cost of slower setpoint changes, disturbance rejection, and increased overshoot.

## Author
This software is written by [Tijs van Roon](https://github.com/knifter). It is free to use under the [MIT License](LICENSE).
