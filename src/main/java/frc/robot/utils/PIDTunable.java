package frc.robot.utils;

/**
 * PIDTunable
 * 
 * A contract that any subsystem can implement to opt into
 * the PID tuning system. The TunePID and AutoTuneMultiMotor
 * commands use this interface so they work generically across
 * any subsystem without needing to know the hardware details.
 * 
 * Place this file at:
 *   src/main/java/frc/robot/util/PIDTunable.java
 * 
 * To use: add "implements PIDTunable" to your subsystem class
 * and add the three @Override methods.
 */
public interface PIDTunable {

    /**
     * Returns the current measurement for this subsystem.
     * Examples:
     *   Intake    -> getCurrPos()
     *   Flywheels -> getLMotorRPM()
     *   Hopper    -> getBeltMotorVelocity()
     */
    double getMeasurement();

    /**
     * Accepts a PID output value and applies it to the motor(s).
     * The tuning command calls this every loop with the computed output.
     */
    void setOutput(double output);

    /**
     * Returns a short name used to namespace SmartDashboard keys.
     * Example: "Intake" produces keys like "Intake/kP", "Intake/Error"
     */
    String getTunableName();
}
