import math
import prefs

class FPID:
    def __init__(self, settings, input_val, output_val):
        """
        Initialize an FPID controller with the provided settings.

        Args:
            settings (dict): A dictionary containing preprocessor feature keys (e.g., 'FPID_ROTATIONAL', 'FPID_FORWARD_LINEAR', etc.) and their associated values.
            input_val (list or iterable): The input value(s) for the controller.
            output_val (list or iterable): The output value(s) for the controller.
        """
        # Store the provided settings dictionary
        self.settings = settings
        self.DEBUG = prefs.get('DEBUG', 0)

        # Store the input and output values
        self.input_val = input_val
        self.output_val = output_val

        # Initialize variables for output limits and ramp rate
        self.min_output = float('-inf')       # Minimum allowable output value
        self.max_output = float('inf')        # Maximum allowable output value
        self.output_ramp_rate = float('inf')  # Maximum rate at which output can increase per cycle

        # Initialize variables for setpoint range and maximum I output
        self.setpoint_range = float('inf')    # Allowable range for the setpoint value
        self.max_i_output = float('inf')      # Maximum value contributed by the integral component

        # Initialize variables for control terms and state variables
        self.error_sum = prefs.get('error_sum', 0)            # Cumulative error sum for the integral component
        self.prev_input = math.nan            # Previous input value
        self.prev_setpoint = math.nan         # Previous setpoint value
        self.prev_output = math.nan           # Previous output value
        self.prev_d_term = math.nan           # Previous derivative term value

        # Initialize variables for Take-Back-Half (TBH) feature
        self.tbh_prev_error_sum = math.nan    # Previous error sum for TBH
        self.tbh_limit = math.nan            # Limit value used in TBH feature

        # Variable to track if the output has been clamped by min/max values
        self.output_clamped_by_min_max = 0   # Initialized to 0 (not clamped)
        self.output_clamped_by_ramp_rate = 0   # Initialized to 0 (not clamped)

    def align_output(self):
        """
        Align the internal state of the controller with the current output.

        This method is typically called before (re)starting the PID loop to prevent a large start-up kick in the output.

        Returns:
            None
        """
        # Reset internal state variables to NaN to mark the first run
        self.prev_input = math.nan
        self.prev_output = math.nan

        # Reset TBH-related variables if TBH feature is enabled
        if 'FPID_TAKEBACKHALF' in self.settings:
            self.tbh_prev_error_sum = math.nan
            self.tbh_limit = math.nan

        # Clamp the current output value to the specified min and max output limits
        self.output_clamped_by_min_max = self.clamp(self.output_val[0], self.min_output, self.max_output)

        # Reset the error sum, which will be recovered during the next PID calculation
        self.error_sum = math.nan

    def set_max_i_output(self, maximum):
        """
        Set the maximum output value contributed by the integral (I) component of the controller.

        This method can be used to prevent large wind-up issues and simplify tuning by limiting the maximum
        effect of the integral term on the output.

        Args:
            maximum (float): The maximum output value contributed by the integral component. Units should
                be the same as the expected output value.

        Returns:
            None
        """
        # Internally max_error and I_zone are similar but scaled for different purposes.
        # The max_error is generated for simplifying math, as calculations against the max error are more common.
        self.max_i_output = maximum

    def set_output_limits(self, minimum, maximum=None):
        """
        Set the output limits for the controller.

        This method allows you to specify the minimum and maximum allowable output values for the controller.
        If only one value is provided, it sets both the minimum and maximum to the same value, effectively
        limiting the controller's output to that value.

        Args:
            minimum (float): The minimum allowable output value.
            maximum (float, optional): The maximum allowable output value. If not provided, it is set to the
                absolute value of the minimum value.

        Returns:
            None
        """
        if maximum is None:
            maximum = abs(minimum)

        # Ensure the minimum and maximum are correctly assigned based on their values
        if maximum > minimum:
            self.max_output = maximum
            self.min_output = minimum
        else:
            self.max_output = minimum
            self.min_output = maximum

        # Ensure the bounds of the integral term are within the bounds of the allowable output swing
        if self.max_i_output == 0 or self.max_i_output > (self.max_output - self.min_output):
            self.set_max_i_output(self.max_output - self.min_output)

    def calculate(self, dt=1.0):
        """Calculate the PID value needed to hit the target setpoint."""
        if not (dt > 0.0) or not math.isfinite(dt):
            if self.DEBUG:
                print("WARN: dt = %f" % dt)
            self.output_val[0] = math.nan
            return False

        # Sample settings for this loop
        setpoint = self.settings['setpoint']
        input_val = self.input_val[0]

        if not math.isfinite(input_val):
            if self.DEBUG:
                print("ERROR: input: NAN")
            self.output_val[0] = math.nan
            return False

        # Ramp the setpoint used for calculations if user has opted to do so
        self.clamp(setpoint, input_val - self.settings['setpoint_range'], input_val + self.settings['setpoint_range'])

        # Calculate error
        error = setpoint - input_val

        # Handle optional rotational correction based on preprocessor directive
        if 'FPID_ROTATIONAL' in self.settings:
            # Rotational correction, error can never be bigger than +-180
            # and we also need to move to the shortes rotation side
            if error > 180:
                error -= 360
            if error < -180:
                error += 360

        # Initialize forward terms
        Foutput_linear = 0
        Foutput_dsetpoint = 0

        # FORWARD TERMS
        # Calculate F output. Notice, this depends only on the setpoint, and not the error.
        if 'FPID_FORWARD_LINEAR' in self.settings:
            Foutput_linear = self.settings['kF'] * (setpoint - self.settings['kF_offset'])

        if 'FPID_FORWARD_DSETPOINT' in self.settings:
            # Calculate Derivative-on-setpoint (which forward term as it depens solely on SP)
            if math.isnan(self.prev_setpoint):
                self.prev_setpoint = setpoint

            Foutput_dsetpoint = self.settings['kDsetpoint'] * (setpoint - self.prev_setpoint)
            self.prev_setpoint = setpoint

        # PROPORTIONAL TERMS
        Poutput = self.settings['kP'] * error

        # Handle optional proportional root term based on preprocessor directive
        PRoutput = 0
        if 'FPID_P_ROOT' in self.settings:
            PRoutput = self.settings['kPR'] * math.sqrt(error)

        # DERIVATIVE TERMS
        # First run/sync
        if math.isnan(self.prev_input):
            self.prev_input = input_val

        # Calculate Derrivative-on-measurement
        # Note, this is negative. this actually "slows" the system if it's doing
        # the correct thing, and small values helps prevent output spikes and overshoot
        # D-term filter can be applied as well if PID loop runs fast and input changes (discretely) slow
        dterm = -1 * (input_val - self.prev_input) / dt

        if math.isnan(self.prev_d_term):
            self.prev_d_term = dterm

        if 'FPID_D_FILTER' in self.settings:
            dterm = dterm * (1 - self.settings['D_filter']) + self.prev_d_term * self.settings['D_filter']

        Doutput = self.settings['kD'] * dterm

        # Handle optional double derivative term based on preprocessor directive
        DDoutput = 0
        if 'FPID_DOUBLE_D' in self.settings:
            ddterm = (dterm - self.prev_d_term) / dt
            DDoutput = self.settings['kDD'] * ddterm

        self.prev_d_term = dterm
        self.prev_input = input_val

        # Calculate integral term with windup prevention based on preprocessor directive
        # /*********** INTEGRAL TERMS ***********************************************************************************/
        # //The Iterm is more complex. There's several things to factor in to make it easier to deal with.
        # // 1. The multiplication with I-gain is done when adding to the sum, this prevents the bump on I-gain changes
        # // 2. prevent further windup by not increasing error_sum if output=max_output or output=min_output
        # // 3. prevent windup by not increasing error_sum if we're already running against our max Ioutput
        # // 3b. But only if the outputclamp and error have the same sign (direction)
        freeze_integral = self.output_clamped_by_min_max and self.output_clamped_by_min_max * error > 0

        if self.DEBUG:
            print("freeze = %d, ramp(%d), minmax(%d) (%f)" % (
                freeze_integral,
                self.output_clamped_by_ramp_rate,
                self.output_clamped_by_min_max,
                self.output_clamped_by_min_max*error
                )
            )

        if 'FPID_OUTPUT_RAMPRATE' in self.settings:
            if self.output_clamped_by_ramp_rate:
                freeze_integral = True

        # If error_sum is NAN we need to align it with the current _output not cause a big jump
        if math.isnan(self.error_sum):
            self.error_sum = 0
            # If there is a previous output, it must be the result of the
            # intergral term + forward terms, we can run that backwards
            # and recover error_sum, otherwise we start with 0 integral now
            if math.isfinite(self.prev_output):
                self.error_sum = self.prev_output
                if 'FPID_FORWARD_LINEAR' in self.settings:
                    self.error_sum = self.error_sum - Foutput_linear
            # FPID_FORWARD_DSETPOINT is not needed as it is a D-term and can just be added this cycle

        # If all good, increase integral
        if not freeze_integral:
            self.error_sum += self.settings['kI'] * error * dt

        if 'FPID_TAKEBACKHALF' in self.settings:
            error_hyst = setpoint * 0.01 # FPID_TBH_HYSTERESIS

            # first loop detection
            if math.isnan(self.tbh_prev_error_sum):
                if error > 0:
                    # If error positive, output will rise, integral will grow
                    self.tbh_limit = +error_hyst
                    self.tbh_prev_error_sum = 0
                else:
                    # If error positive, output will fall, integral will shrink
                    self.tbh_limit = -error_hyst
                    self.tbh_prev_error_sum = self.error_sum
            if self.DEBUG:
                print("TBH: init, limit = %.02f, sum_prv = %.02f" % (
                    self.tbh_limit,
                    self.tbh_prev_error_sum)
                )

            # if zero crossing (with hysteresis), average the intergral between current value
            # and value of last crossing: Take Back Half
            if (self.tbh_limit > 0 and error > self.tbh_limit):
                if self.DEBUG:
                    print("TBH: crossing from - to +, prv_I = %f" % (self.tbh_prev_error_sum))
                self.tbh_limit = -error_hyst
                self.error_sum = self.tbh_prev_error_sum = (self.error_sum + self.tbh_prev_error_sum) / 2

            if (self.tbh_limit < 0 and error < self.tbh_limit):
                if self.DEBUG:
                    print("TBH: crossing from + to -, prv_I = %f" % (self.tbh_prev_error_sum))
                self.tbh_limit = +error_hyst
                self.error_sum = self.tbh_prev_error_sum = (self.error_sum + self.tbh_prev_error_sum) / 2

        # 3. max_i_output restricts the amount of output contributed by the Iterm.
        self.clamp(self.error_sum, -1 * self.settings['max_i_output'], self.settings['max_i_output'])

        # Now our I output term is just the sum as the I factor is already processed while adding to the sum previously
        Ioutput = self.error_sum if self.settings['kI'] > 0.0 else 0.0

        # And, finally, we can just add the terms up
        output = Poutput + Ioutput + Doutput + Foutput_linear + Foutput_dsetpoint + PRoutput + DDoutput

        if self.DEBUG:
            print("in:%f error = %.2f FPID = F:%f + P:%f + I:%f%s + D:%f = %f" % (
              input_val,
              error,
              Foutput_linear + Foutput_dsetpoint,
              Poutput + PRoutput,
              Ioutput,
              "(frozen)" if freeze_integral else "",
              Doutput,
              output)
            )

        if 'FPID_OUTPUT_RAMPRATE' in self.settings:
            # First run/sync:
            # WARNING: if ramprate limited and isnan(_output_ptr), we'll keep outputting NaN, as that makes sense if ramprate is limited to prevent a big jump at turn-on.
            if math.isnan(self.prev_output):
                self.prev_output = output

            # Limit the output by min/max_output
            self.output_clamped_by_ramp_rate = self.clamp(output, self.prev_output - self.settings['output_ramp_rate'], self.prev_output + self.settings['output_ramp_rate'])
        else:
            # First run/sync, if no ramprate limit we can just set the output to the first value
            if not math.isfinite(self.prev_output):
                self.prev_output = output

        # Limit the output by min/max_output
        self.output_clamped_by_min_max = self.clamp(output, self.settings['min_output'], self.settings['max_output'])

        # Filter the Output: 1 = old output (low-pass), 0 = new output, -1 = subtract old output (high-pass, not implemented yet)
        if 'FPID_OUTPUT_FILTER' in self.settings and not math.isnan(self.prev_output):
            output = output * (1 - self.settings['output_filter']) + self.prev_output * self.settings['output_filter']

        self.prev_output = output
        self.output_val[0] = output

        prefs.set('error_sum', int(self.error_sum * 100) / 100)

        return not freeze_integral

    @staticmethod
    def clamp(value, min_val, max_val):
        """Force a value into a specific range."""
        if value > max_val:
            value = max_val
            return 1
        if value < min_val:
            value = min_val
            return -1
        return 0
