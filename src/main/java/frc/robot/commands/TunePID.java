package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PIDTunable;

/**
 * TunePID — Manual PID Tuning Command
 * 
 * Allows live tuning of any subsystem that implements PIDTunable.
 * kP, kI, kD, and Setpoint are all editable from SmartDashboard
 * while the command is running — no redeployment needed.
 * 
 * Usage in RobotContainer:
 *   SmartDashboard.putData("Tune Intake PID",   new TunePID(m_intake,   m_intake));
 *   SmartDashboard.putData("Tune Launcher PID", new TunePID(m_launcher, m_launcher));
 *   SmartDashboard.putData("Tune Hopper PID",   new TunePID(m_feeder,   m_feeder));
 * 
 * Because addRequirements() is called, only one TunePID can run at a time.
 * Clicking a second tuning button on the dashboard automatically cancels the first.
 * 
 * SmartDashboard keys (namespaced per subsystem, e.g. "Intake/kP"):
 *   <Name>/kP          — proportional gain (editable)
 *   <Name>/kI          — integral gain (editable)
 *   <Name>/kD          — derivative gain (editable)
 *   <Name>/Setpoint    — target value (editable)
 *   <Name>/Measurement — current sensor reading (read only)
 *   <Name>/Error       — setpoint minus measurement (read only)
 *   <Name>/Output      — PID output sent to motor (read only)
 *   <Name>/AtSetpoint  — true when within tolerance (read only)
 * 
 * Place this file at:
 *   src/main/java/frc/robot/commands/TunePID.java
 */
public class TunePID extends Command {

    private final PIDTunable target;
    private final PIDController pid;
    private final String name;

    /**
     * @param target    The subsystem to tune (must implement PIDTunable)
     * @param subsystem The same subsystem cast as SubsystemBase (for requirements)
     * 
     * They are the same object — Java requires both types explicitly.
     * Example: new TunePID(m_intake, m_intake)
     */
    public TunePID(PIDTunable target, SubsystemBase subsystem) {
        this.target = target;
        this.name   = target.getTunableName();
        this.pid    = new PIDController(0, 0, 0);

        addRequirements(subsystem);

        // Initialize SmartDashboard keys so they appear immediately
        SmartDashboard.putNumber(name + "/kP",       0.0);
        SmartDashboard.putNumber(name + "/kI",       0.0);
        SmartDashboard.putNumber(name + "/kD",       0.0);
        SmartDashboard.putNumber(name + "/Setpoint", 0.0);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setPID(
            SmartDashboard.getNumber(name + "/kP", 0.0),
            SmartDashboard.getNumber(name + "/kI", 0.0),
            SmartDashboard.getNumber(name + "/kD", 0.0)
        );
    }

    @Override
    public void execute() {
        // Pull gains live every loop — changes on dashboard take effect immediately
        pid.setPID(
            SmartDashboard.getNumber(name + "/kP", 0.0),
            SmartDashboard.getNumber(name + "/kI", 0.0),
            SmartDashboard.getNumber(name + "/kD", 0.0)
        );

        double setpoint    = SmartDashboard.getNumber(name + "/Setpoint", 0.0);
        double measurement = target.getMeasurement();
        double output      = pid.calculate(measurement, setpoint);

        target.setOutput(output);

        // Publish live feedback
        SmartDashboard.putNumber (name + "/Measurement", measurement);
        SmartDashboard.putNumber (name + "/Error",       pid.getPositionError());
        SmartDashboard.putNumber (name + "/Output",      output);
        SmartDashboard.putBoolean(name + "/AtSetpoint",  pid.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the motor when tuning ends
        target.setOutput(0);
        SmartDashboard.putBoolean(name + "/AtSetpoint", false);
    }

    // Runs until cancelled — does not finish on its own
    @Override
    public boolean isFinished() {
        return false;
    }
}
