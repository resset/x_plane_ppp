import pid


SAMPLE_TIME_S = 0.01

# Maximum run-time of simulation
SIMULATION_TIME_MAX = 4.0


def test(inp):
    ALPHA = 0.02

    test.output = (SAMPLE_TIME_S * inp + test.output) / (1.0 + ALPHA * SAMPLE_TIME_S)

    return test.output
test.output = 0.0


if __name__ == "__main__":
    p = pid.pid(
        kp=2.0,
        ki=0.5,
        kd=-0.25,
        tau=0.01,
        out_limit_min=-10.0,
        out_limit_max=10.0,
        t=0.01
    )

    setpoint = 1.0

    print("Time (s)\tSystem Output\tControllerOutput")
    t = 0.0
    while t < SIMULATION_TIME_MAX:
        # Get measurement from system
        measurement = test(p.output)

        # Compute new control signal
        p.update(setpoint, measurement)

        print("%f\t%f\t%f" % (t, measurement, p.output))

        t += SAMPLE_TIME_S
