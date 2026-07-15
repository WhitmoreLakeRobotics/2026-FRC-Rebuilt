package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MultiMotorPIDTunable;

/**
 * AutoTuneMultiMotor — Automatic PID Tuning Command (Ziegler-Nichols)
 * 
 * Automatically finds PID gain recommendations for each motor in a subsystem
 * by running a Ziegler-Nichols ultimate gain test. Tunes one motor at a time,
 * then publishes recommended kP/kI/kD values to SmartDashboard when complete.
 * 
 * How Ziegler-Nichols works:
 *   1. Sets kI and kD to zero, slowly ramps up kP
 *   2. Detects when the system starts oscillating consistently (ultimate gain Ku)
 *   3. Measures the oscillation period (Tu)
 *   4. Calculates recommendations:
 *        kP = 0.6  * Ku
 *        kI = 1.2  * Ku / Tu
 *        kD = 0.075 * Ku * Tu
 * 
 * NOTE on kF (feedforward):
 *   kF cannot be found by Ziegler-Nichols — it is a feed-forward term, not
 *   feedback. Characterize kF separately using WPILib SysId, then plug the
 *   result in. The auto-tuner leaves kF as 0 in its output.
 * 
 * NOTE on safety:
 *   The mechanism WILL oscillate during tuning. Run in open space with
 *   software limits enabled. Have a driver ready to disable the robot.
 *   Treat all recommended values as starting points — back off kP and kD
 *   by ~20% before competition use.
 * 
 * Usage in RobotContainer:
 *   // LFlywheels: one setpoint used for both motors
 *   SmartDashboard.putData("AutoTune Flywheels",
 *       new AutoTuneMultiMotor(m_launcher.flywheels_LH, m_launcher.flywheels_LH, 3750.0));
 * 
 * SmartDashboard keys after completion (example for LFlywheels):
 *   LFlywheels/AutoTune/Status
 *   LFlywheels/Left/Recommended_kP
 *   LFlywheels/Left/Recommended_kI
 *   LFlywheels/Left/Recommended_kD
 *   LFlywheels/Right/Recommended_kP
 *   LFlywheels/Right/Recommended_kI
 *   LFlywheels/Right/Recommended_kD
 *   LFlywheels/AutoTune/Summary   (human-readable final output)
 * 
 * Place this file at:
 *   src/main/java/frc/robot/commands/AutoTuneMultiMotor.java
 */
public class AutoTuneMultiMotor extends Command {

    // -----------------------------------------------------------------------
    // Tuning parameters — adjust these for your mechanism
    // -----------------------------------------------------------------------

    /** How much kP increases per ramp step. Smaller = more precise but slower. */
    private static final double KP_STEP = 0.0001;

    /** Seconds to wait at each kP level before deciding if oscillation has started. */
    private static final double RAMP_SETTLE_TIME = 1.5;

    /** Number of consistent oscillation cycles required before trusting the period. */
    private static final int REQUIRED_OSCILLATIONS = 4;

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------

    private enum TuningPhase { RAMPING, DETECTING_OSCILLATION, DONE }

    private final MultiMotorPIDTunable target;
    private final double tuningSetpoint;

    // Per-motor result storage
    private final double[] recommended_kP;
    private final double[] recommended_kI;
    private final double[] recommended_kD;

    // Current motor being tuned
    private int currentMotor = 0;
    private TuningPhase phase = TuningPhase.RAMPING;

    // Oscillation detection state (reset between motors)
    private double kP_current    = KP_STEP;
    private double ultimateGain  = 0;
    private double oscillationPeriod = 0;
    private double lastMeasurement   = 0;
    private double lastPeakTime      = 0;
    private int    oscillationCount  = 0;
    private double periodSum         = 0;
    private boolean goingUp          = true;
    private double phaseStartTime    = 0;

    /**
     * @param target          The subsystem to tune (must implement MultiMotorPIDTunable)
     * @param subsystem       The same subsystem cast as SubsystemBase (for requirements)
     * @param tuningSetpoint  The target value to use during tuning
     *                        (e.g. 3750 RPM for flywheels, 90 degrees for an arm)
     */
    public AutoTuneMultiMotor(MultiMotorPIDTunable target,
                               SubsystemBase subsystem,
                               double tuningSetpoint) {
        this.target          = target;
        this.tuningSetpoint  = tuningSetpoint;

        int count = target.getMotorCount();
        recommended_kP = new double[count];
        recommended_kI = new double[count];
        recommended_kD = new double[count];

        addRequirements(subsystem);
    }

    // -----------------------------------------------------------------------
    // Command lifecycle
    // -----------------------------------------------------------------------

    @Override
    public void initialize() {
        currentMotor = 0;
        resetMotorTuningState();
        updateStatus("Starting — tuning motor: " + target.getMotorName(0));
    }

    @Override
    public void execute() {
        double now         = Timer.getFPGATimestamp();
        double measurement = target.getMeasurement(currentMotor);
        String motorLabel  = target.getTunableName() + "/" + target.getMotorName(currentMotor);

        switch (phase) {

            case RAMPING:
                // Drive motor with P-only control at current kP
                target.setOutput(currentMotor, pOnly(measurement));

                if (now - phaseStartTime > RAMP_SETTLE_TIME) {
                    if (isOscillating(measurement)) {
                        // Found the ultimate gain — start measuring the period
                        ultimateGain    = kP_current;
                        oscillationCount = 0;
                        periodSum        = 0;
                        lastPeakTime     = 0;
                        goingUp          = true;
                        phase            = TuningPhase.DETECTING_OSCILLATION;
                        updateStatus("Oscillation found at kP=" + String.format("%.5f", kP_current)
                                + " — measuring period for " + target.getMotorName(currentMotor));
                    } else {
                        // Not oscillating yet — increase kP and wait again
                        kP_current   += KP_STEP;
                        phaseStartTime = now;
                        updateStatus("Ramping kP=" + String.format("%.5f", kP_current)
                                + " on " + target.getMotorName(currentMotor));
                    }
                }
                break;

            case DETECTING_OSCILLATION:
                // Hold at ultimate gain and count oscillation cycles
                target.setOutput(currentMotor, pOnly(measurement));
                detectPeaks(measurement, now);

                SmartDashboard.putNumber(motorLabel + "/OscillationCount", oscillationCount);

                if (oscillationCount >= REQUIRED_OSCILLATIONS) {
                    // Enough cycles — calculate and store gains for this motor
                    oscillationPeriod = periodSum / oscillationCount;
                    calculateAndStore(currentMotor);
                    publishMotorResults(currentMotor);

                    // Stop this motor before moving to the next
                    target.setOutput(currentMotor, 0);

                    currentMotor++;
                    if (currentMotor < target.getMotorCount()) {
                        resetMotorTuningState();
                        updateStatus("Moving to motor: " + target.getMotorName(currentMotor));
                    } else {
                        phase = TuningPhase.DONE;
                        applyAllGains();
                        publishFinalSummary();
                    }
                }
                break;

            case DONE:
                // Stop everything — results are on SmartDashboard
                for (int i = 0; i < target.getMotorCount(); i++) {
                    target.setOutput(i, 0);
                }
                break;
        }

        // Live telemetry for the motor currently being tuned
        SmartDashboard.putNumber(motorLabel + "/Measurement", measurement);
        SmartDashboard.putNumber(motorLabel + "/CurrentKp",   kP_current);
        SmartDashboard.putNumber(motorLabel + "/Error",       tuningSetpoint - measurement);

        lastMeasurement = measurement;
    }

    @Override
    public boolean isFinished() {
        return phase == TuningPhase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < target.getMotorCount(); i++) {
            target.setOutput(i, 0);
        }
        if (interrupted) {
            updateStatus("Interrupted at kP=" + String.format("%.5f", kP_current)
                    + " on motor " + target.getMotorName(currentMotor));
        }
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /** P-only output — kI and kD are zero during tuning phases. */
    private double pOnly(double measurement) {
        return kP_current * (tuningSetpoint - measurement);
    }

    /** Returns true if the measurement just crossed the setpoint. */
    private boolean isOscillating(double measurement) {
        return (lastMeasurement < tuningSetpoint) != (measurement < tuningSetpoint);
    }

    /** Tracks peaks to measure oscillation period. */
    private void detectPeaks(double measurement, double now) {
        if (goingUp && measurement < lastMeasurement) {
            // Just passed a peak
            if (lastPeakTime > 0) {
                periodSum += (now - lastPeakTime);
                oscillationCount++;
            }
            lastPeakTime = now;
            goingUp = false;
        } else if (!goingUp && measurement > lastMeasurement) {
            // Just passed a trough
            goingUp = true;
        }
    }

    /** Applies Ziegler-Nichols formulas and stores results for this motor. */
    private void calculateAndStore(int motorIndex) {
        double Ku = ultimateGain;
        double Tu = oscillationPeriod;
        recommended_kP[motorIndex] = 0.6   * Ku;
        recommended_kI[motorIndex] = 1.2   * Ku / Tu;
        recommended_kD[motorIndex] = 0.075 * Ku * Tu;
    }

    /** Writes recommended gains to each motor controller (no flash persist). */
    private void applyAllGains() {
        for (int i = 0; i < target.getMotorCount(); i++) {
            target.applyGains(i,
                recommended_kP[i],
                recommended_kI[i],
                recommended_kD[i],
                0.0); // kF left as 0 — characterize separately with SysId
        }
    }

    /** Publishes results for one motor as they become available. */
    private void publishMotorResults(int motorIndex) {
        String label = target.getTunableName() + "/" + target.getMotorName(motorIndex);
        SmartDashboard.putNumber(label + "/Recommended_kP", recommended_kP[motorIndex]);
        SmartDashboard.putNumber(label + "/Recommended_kI", recommended_kI[motorIndex]);
        SmartDashboard.putNumber(label + "/Recommended_kD", recommended_kD[motorIndex]);
        SmartDashboard.putNumber(label + "/UltimateGain_Ku", ultimateGain);
        SmartDashboard.putNumber(label + "/OscillationPeriod_Tu", oscillationPeriod);
    }

    /** Publishes a human-readable summary of all motors when fully complete. */
    private void publishFinalSummary() {
        String base = target.getTunableName() + "/AutoTune";
        updateStatus("COMPLETE — see recommendations below");

        StringBuilder summary = new StringBuilder();
        for (int i = 0; i < target.getMotorCount(); i++) {
            summary.append(target.getMotorName(i))
                   .append(": kP=").append(String.format("%.5f", recommended_kP[i]))
                   .append(" kI=").append(String.format("%.5f", recommended_kI[i]))
                   .append(" kD=").append(String.format("%.5f", recommended_kD[i]))
                   .append(" | ");
        }
        SmartDashboard.putString(base + "/Summary", summary.toString());
    }

    /** Resets all oscillation detection state for the next motor. */
    private void resetMotorTuningState() {
        phase            = TuningPhase.RAMPING;
        kP_current       = KP_STEP;
        ultimateGain     = 0;
        oscillationPeriod = 0;
        oscillationCount = 0;
        periodSum        = 0;
        lastPeakTime     = 0;
        goingUp          = true;
        phaseStartTime   = Timer.getFPGATimestamp();
        lastMeasurement  = target.getMeasurement(currentMotor);
    }

    private void updateStatus(String msg) {
        SmartDashboard.putString(target.getTunableName() + "/AutoTune/Status", msg);
    }
}
