package frc.robot.utils.motor;

/**
 * A closed loop that controls position, e.g. an arm, elevator or extender.
 * Units are whatever the settings' position factor gives (motor rotations by default).
 */
public interface PositionLoop extends ClosedLoop {

    double getPosition();

    /** Commands the loop to move to and hold this position (clamped to [min, max]). */
    void setPosition(double position);

    double getTargetPosition();

    /** Lowest position the mechanism may be commanded to; tuners never step outside [min, max]. */
    double minPosition();

    double maxPosition();

    /** Defines the current position as `position` (e.g. 0 at a hard stop). */
    void resetPosition(double position);

    default boolean atPosition(double tolerance) {
        return Math.abs(getPosition() - getTargetPosition()) <= tolerance;
    }
}
