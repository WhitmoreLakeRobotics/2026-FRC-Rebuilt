package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PIDTunable;

/**
 * TunePID — Manual PID Tuning Command
 *
 * Allows live tuning of any subsystem that implements PIDTunable.
 * kP, kI, kD, kV, and Setpoint are all editable from SmartDashboard
 * while the command is running — no redeployment needed.
 *
 * The PID loop runs on the motor controller. This command only pushes the
 * dashboard gains to it (when they change) and commands the setpoint.
 *
 * Usage in RobotContainer:
 *   SmartDashboard.putData("Tune Intake PID",   new TunePID(m_intake,   m_intake));
 *   SmartDashboard.putData("Tune Launcher PID", new TunePID(m_launcher, m_launcher));
 *
 * Because addRequirements() is called, only one TunePID can run at a time.
 * Clicking a second tuning button on the dashboard automatically cancels the first.
 *
 * SmartDashboard keys (namespaced per subsystem, e.g. "Intake/kP"):
 *   <Name>/kP          — proportional gain (editable)
 *   <Name>/kI          — integral gain (editable)
 *   <Name>/kD          — derivative gain (editable)
 *   <Name>/kV          — velocity feedforward (editable)
 *   <Name>/Setpoint    — target value (editable)
 *   <Name>/Measurement — current sensor reading (read only)
 *   <Name>/Error       — setpoint minus measurement (read only)
 *   <Name>/AtSetpoint  — true when within 2% of the setpoint (read only)
 *
 * Place this file at:
 *   src/main/java/frc/robot/commands/TunePID.java
 */
public class TunePID extends Command {

    private static final double AT_SETPOINT_FRACTION = 0.02;

    private final PIDTunable target;
    private final String name;

    private double[] lastGains = new double[4];

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

        addRequirements(subsystem);

        // Initialize SmartDashboard keys so they appear immediately,
        // starting from the gains currently on the controller
        double[] gains = target.getPID();
        SmartDashboard.putNumber(name + "/kP",       gains[0]);
        SmartDashboard.putNumber(name + "/kI",       gains[1]);
        SmartDashboard.putNumber(name + "/kD",       gains[2]);
        SmartDashboard.putNumber(name + "/kV",       gains[3]);
        SmartDashboard.putNumber(name + "/Setpoint", 0.0);
    }

    @Override
    public void initialize() {
        // Start from the gains actually on the controller (e.g. left there by AutoTuneVelocity)
        double[] gains = target.getPID();
        SmartDashboard.putNumber(name + "/kP", gains[0]);
        SmartDashboard.putNumber(name + "/kI", gains[1]);
        SmartDashboard.putNumber(name + "/kD", gains[2]);
        SmartDashboard.putNumber(name + "/kV", gains[3]);

        // Force the first execute() to push the dashboard gains
        lastGains = new double[] { Double.NaN, Double.NaN, Double.NaN, Double.NaN };
    }

    @Override
    public void execute() {
        // Pull gains live every loop, but only touch the controller when one changed
        double[] gains = {
            SmartDashboard.getNumber(name + "/kP", 0.0),
            SmartDashboard.getNumber(name + "/kI", 0.0),
            SmartDashboard.getNumber(name + "/kD", 0.0),
            SmartDashboard.getNumber(name + "/kV", 0.0)
        };
        if (!java.util.Arrays.equals(gains, lastGains)) {
            target.applyPID(gains[0], gains[1], gains[2], gains[3]);
            lastGains = gains;
        }

        double setpoint    = SmartDashboard.getNumber(name + "/Setpoint", 0.0);
        double measurement = target.getMeasurement();
        double error       = setpoint - measurement;

        target.setSetpoint(setpoint);

        // Publish live feedback
        SmartDashboard.putNumber (name + "/Measurement", measurement);
        SmartDashboard.putNumber (name + "/Error",       error);
        SmartDashboard.putBoolean(name + "/AtSetpoint",  Math.abs(error) <= Math.abs(setpoint) * AT_SETPOINT_FRACTION);
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the motor when tuning ends
        target.setSetpoint(0);
        SmartDashboard.putBoolean(name + "/AtSetpoint", false);
    }

    // Runs until cancelled — does not finish on its own
    @Override
    public boolean isFinished() {
        return false;
    }
}
