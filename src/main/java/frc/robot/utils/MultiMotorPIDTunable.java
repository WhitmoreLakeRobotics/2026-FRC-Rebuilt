package frc.robot.utils;

/**
 * MultiMotorPIDTunable
 * 
 * An extended contract for subsystems that have more than one motor,
 * each potentially needing independent PID gains.
 * 
 * Examples:
 *   LFlywheels -> two motors (Left, Right) with separate kP/kI/kD/kF arrays
 *   Climb      -> two motors with different PID slots per direction
 * 
 * Extends PIDTunable so single-motor fallback methods are still available.
 * 
 * Place this file at:
 *   src/main/java/frc/robot/util/MultiMotorPIDTunable.java
 */
public interface MultiMotorPIDTunable extends PIDTunable {

    /**
     * Returns the number of motors to tune independently.
     * Example: LFlywheels returns 2
     */
    int getMotorCount();

    /**
     * Returns a human-readable name for each motor.
     * Used to namespace SmartDashboard keys per motor.
     * Example: motorIndex 0 -> "Left", motorIndex 1 -> "Right"
     */
    String getMotorName(int motorIndex);

    /**
     * Returns the current measurement for a specific motor.
     * Example: motorIndex 0 -> TowerLM.getEncoder().getVelocity()
     */
    double getMeasurement(int motorIndex);

    /**
     * Applies a PID output to a specific motor.
     * Called by the tuner command every loop for the motor currently being tuned.
     */
    void setOutput(int motorIndex, double output);

    /**
     * Writes finalized PID gains to a specific motor controller.
     * Called once per motor after tuning is complete.
     * 
     * Uses kNoPersistParameters during tuning — call persist manually
     * once you are happy with the recommended values.
     */
    void applyGains(int motorIndex, double kP, double kI, double kD, double kF);
}
