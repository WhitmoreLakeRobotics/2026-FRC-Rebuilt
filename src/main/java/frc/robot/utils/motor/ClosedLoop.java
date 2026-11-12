package frc.robot.utils.motor;

/**
 * What every tunable closed loop can do, whatever it controls.
 *
 * VelocityLoop and PositionLoop add the measurement and setpoint for their kind of loop.
 * Tuning commands and dashboards work against these interfaces, so they never see a
 * SparkMax (or whatever controller is behind the loop).
 */
public interface ClosedLoop {

    /** Short name used to namespace dashboard keys, e.g. "Intake/Roller". */
    String name();

    /** Gains currently on the controller. */
    Gains getGains();

    /**
     * Writes gains to the controller without saving them to flash, so a tuning run can try
     * values freely. Copy the final values into the subsystem's settings to keep them.
     */
    void applyGains(Gains gains);

    /** Clears the accumulated integral error so one test's wind-up does not leak into the next. */
    void resetIntegral();

    /** Stops driving the loop (output 0, not a setpoint of 0). */
    void stop();
}
