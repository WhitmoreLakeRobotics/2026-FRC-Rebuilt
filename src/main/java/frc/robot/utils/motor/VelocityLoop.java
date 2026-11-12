package frc.robot.utils.motor;

/**
 * A closed loop that controls speed, e.g. a roller or flywheel.
 * Units are whatever the settings' velocity factor gives (RPM by default).
 */
public interface VelocityLoop extends ClosedLoop {

    double getVelocity();

    /** Commands the loop to hold this speed; 0 stops the motor rather than braking against a setpoint. */
    void setVelocity(double velocity);

    double getTargetVelocity();

    /** True once the speed is within tolerance of the target. */
    default boolean atVelocity(double tolerance) {
        return Math.abs(getVelocity() - getTargetVelocity()) <= tolerance;
    }
}
