package frc.robot.utils;

/**
 * PIDTunable
 *
 * A contract that any subsystem can implement to opt into
 * the PID tuning system. The TunePID and AutoTuneMultiMotor
 * commands use this interface so they work generically across
 * any subsystem without needing to know the hardware details.
 *
 * The PID loop runs on the motor controller (SparkMax), so TunePID does not
 * compute an output itself. It reads kP/kI/kD/kV/Setpoint from SmartDashboard,
 * pushes the gains to the controller through applyPID() and the target through
 * setSetpoint(), so you are tuning the same loop the robot code really uses.
 *
 * Place this file at:
 *   src/main/java/frc/robot/util/PIDTunable.java
 *
 * To use: add "implements PIDTunable" to your subsystem class
 * and add the @Override methods.
 */
public interface PIDTunable {

    /**
     * Returns the current measurement for this subsystem, in the same units
     * as the setpoint.
     * Examples:
     *   Intake roller -> getRollerRPM()
     *   Flywheels     -> getLMotorRPM()
     */
<<<<<<< Updated upstream
    //SparkClosedLoopController getLoopController();

    double getCurrVelocity();
    double getTargVelocity();
    double getP();
    double getI();
    double getD();
    double getF();
    double getIACC();
    
    void setCurrVelocity();
    void setTargVelocity();
    void setP();
    void setI();
    void setD();
    void setF();
    void setIACC();
=======
    double getMeasurement();

    /**
     * Commands the closed loop to this target (e.g. RPM for a velocity loop).
     * TunePID calls this every loop with the dashboard setpoint, and with 0 when it ends.
     */
    void setSetpoint(double setpoint);

    /**
     * Writes new gains to the motor controller's closed loop.
     * Should NOT persist to flash — copy the final values into the subsystem's
     * config once you are happy with them.
     * TunePID only calls this when a dashboard value actually changes.
     */
    void applyPID(double kP, double kI, double kD, double kV);

    /**
     * Returns the gains currently on the controller as {kP, kI, kD, kV}.
     * TunePID uses these as the starting dashboard values.
     */
    double[] getPID();

    /**
     * Clears the controller's accumulated integral error. Auto-tuning calls this before
     * each test so one trial's wound-up integral does not leak into the next.
     * Default does nothing, which is fine for subsystems that are never tuned for kI.
     */
    default void resetIntegral() {}
>>>>>>> Stashed changes

    /**
     * Returns a short name used to namespace SmartDashboard keys.
     * Example: "Intake" produces keys like "Intake/kP", "Intake/Error"
     */
    String getTunableName();
<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
}
