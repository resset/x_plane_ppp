class pid:
    # Controller gains
    kp = 0.0
    ki = 0.0
    kd = 0.0

    # Derivative low-pass filter time constant
    tau = 0.0

    # Output limits
    out_limit_min = 0.0
    out_limit_max = 0.0

    # Sample time
    t = 0.0

    # Storage for controller values
    integrator = 0.0
    previous_error = 0.0
    differentiator = 0.0
    previous_measurement = 0.0

    # Controller output
    output = 0.0

    def __init__(self, kp, ki, kd, tau, out_limit_min, out_limit_max, t):
        super().__init__()
        self.integrator = 0.0
        self.previous_error = 0.0

        self.differentiator = 0.0
        self.previous_measurement = 0.0

        self.output = 0.0

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tau = tau
        self.out_limit_min = out_limit_min
        self.out_limit_max = out_limit_max
        self.t = t

    def update(self, setpoint, measurement):
        error = setpoint - measurement

        proportional = self.kp * error

        self.integrator = self.integrator + 0.5 * self.ki * self.t * (error + self.previous_error)

        # Anti wind-up via dynamic integrator clamping
        # See: https://e2e.ti.com/blogs_/b/industrial_strength/archive/2013/04/13/teaching-your-pi-controller-to-behave-part-vii
        # Compute integrator limits
        out_limit_max_int = 0.0
        if self.out_limit_max > proportional:
            out_limit_max_int = self.out_limit_max - proportional
        else:
            out_limit_max_int = 0.0

        out_limit_min_int = 0.0
        if self.out_limit_min < proportional:
            out_limit_min_int = self.out_limit_min - proportional
        else:
            out_limit_min_int = 0.0

        # Clamp integrator
        if self.integrator > out_limit_max_int:
            self.integrator = out_limit_max_int
        elif self.integrator < out_limit_min_int:
            self.integrator = out_limit_min_int

        # Derivative (band-limited differentiator)
        # self.differentiator = (2.0 * self.kd * (error - self.previous_error)
        #     + (2.0 * self.tau - self.t) * self.differentiator)
        #     / (2.0 * self.tau + self.t);
        # Note: derivative on measurement, therefore minus sign in front of equation!
        # /* Note on 'derivative-on-measurement': Since the 'error signal'
        #  * effectively going into the differentiator does not depend on the
        #  * setpoint: e[n] = 0 - measurement, and therefore
        #  * (e[n] - e[n - 1]) = (0 - measurement) - (0 - prevMeasurement) = -Kd * (measurement - prevMeasurement).
        #  * Note the minus sign compared to derivative-on-error!
        #  * I've made the change in the Git repo - before you would have had to
        #  * use a negative Kd gain to get the same result. Now you can, as normal
        #  * with derivative-on-error, use a positive Kd gain as usual.*/
        self.differentiator = -(2.0 * self.kd * (measurement - self.previous_measurement) \
            + (2.0 * self.tau - self.t) * self.differentiator) \
            / (2.0 * self.tau + self.t)

        # Compute output and apply limits
        self.output = proportional + self.integrator + self.differentiator

        if self.output > self.out_limit_max:
            self.output = self.out_limit_max
        elif self.output < self.out_limit_min:
            self.output = self.out_limit_min

        # Store error and measurement for later use
        self.previous_error = error
        self.previous_measurement = measurement

        return self.output
