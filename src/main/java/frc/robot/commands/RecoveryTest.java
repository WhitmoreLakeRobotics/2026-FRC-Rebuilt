package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PIDTunable;

/**
 * RecoveryTest — measures how a velocity loop recovers from a real load. It changes no gains.
 *
 * Spins the mechanism up to the setpoint with whatever gains are on the controller, then waits
 * for you to load it (grab the roller, feed a game piece). Each load is one "shot":
 *
 *   dip depth      how far the speed fell below the setpoint (fraction of setpoint)
 *   recovery time  seconds from the start of the dip until the speed is back within
 *                  RECOVER_BAND of the setpoint and stays there for RECOVER_HOLD
 *   overshoot      the highest speed above the setpoint in the OVERSHOOT_WINDOW after recovering
 *
 * Run it before and after AutoTuneVelocity, or while hand-editing gains in TunePID's dashboard,
 * to compare gains with the same kind of load.
 *
 * SAFETY: the mechanism spins at the setpoint until all shots are done or the command is
 * cancelled. Keep hands clear of pinch points and only load it the way the game would.
 *
 * SmartDashboard keys (e.g. "Intake/Recovery/..."):
 *   Status, Shots, Last_DipDepth, Last_RecoveryTime, Last_Overshoot, Worst_RecoveryTime, Measurement
 */
public class RecoveryTest extends Command {

    /** A dip starts when the speed falls this far below the setpoint. */
    static final double DIP_TRIGGER = 0.05;
    /** Recovered when within this fraction of the setpoint... */
    static final double RECOVER_BAND = 0.03;
    /** ...for this many seconds without leaving the band. */
    static final double RECOVER_HOLD = 0.10;
    /** Give up on a shot that has not recovered after this long. */
    static final double RECOVER_TIMEOUT = 5.0;
    /** Time after recovering during which overshoot is measured. */
    static final double OVERSHOOT_WINDOW = 0.5;
    /** Time to reach the setpoint before the first load is accepted. */
    static final double SPINUP_TIME = 3.0;
    /** Give up if no load is detected within this long after spin-up. */
    static final double WAIT_TIMEOUT = 60.0;

    private enum Phase { SPINUP, ARMED, RECOVERING, TAIL, DONE }

    private final PIDTunable target;
    private final String name;
    private final double setpoint;
    private final double dir;
    private final int totalShots;

    private Phase phase;
    private double phaseStart;
    private int shots;
    private double dipStart;
    private double minSpeed;
    private double peakSpeed;
    private double recoverTime;
    private double inBandSince;
    private double worstRecovery;
    private boolean failed;

    /**
     * @param target    The subsystem to test (must implement PIDTunable)
     * @param subsystem The same subsystem cast as SubsystemBase (for requirements)
     * @param setpoint  The velocity to hold while loading it
     * @param shots     How many loads to measure before finishing
     */
    public RecoveryTest(PIDTunable target, SubsystemBase subsystem, double setpoint, int shots) {
        this.target     = target;
        this.name       = target.getTunableName() + "/Recovery";
        this.setpoint   = setpoint;
        this.dir        = Math.signum(setpoint);
        this.totalShots = shots;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        shots = 0;
        worstRecovery = 0;
        failed = false;
        target.resetIntegral();
        target.setSetpoint(setpoint);
        setPhase(Phase.SPINUP);
        publishCounts();
        status("Spinning up to " + String.format("%.0f", setpoint));
    }

    @Override
    public void execute() {
        double now     = Timer.getFPGATimestamp();
        double elapsed = now - phaseStart;
        double speed   = target.getMeasurement() * dir;
        double sp      = Math.abs(setpoint);

        SmartDashboard.putNumber(name + "/Measurement", speed * dir);
        target.setSetpoint(setpoint);

        switch (phase) {

            case SPINUP:
                if (elapsed >= SPINUP_TIME) {
                    setPhase(Phase.ARMED);
                    status("Ready — load the mechanism (shot " + (shots + 1) + " of " + totalShots + ")");
                }
                break;

            case ARMED:
                if (speed < (1 - DIP_TRIGGER) * sp) {
                    dipStart = now;
                    minSpeed = speed;
                    inBandSince = Double.NaN;
                    setPhase(Phase.RECOVERING);
                    status("Load detected — measuring recovery");
                } else if (elapsed >= WAIT_TIMEOUT) {
                    failed = true;
                    status("FAILED: no load detected within " + (int) WAIT_TIMEOUT + " s");
                    phase = Phase.DONE;
                }
                break;

            case RECOVERING:
                minSpeed = Math.min(minSpeed, speed);
                if (Math.abs(speed - sp) <= RECOVER_BAND * sp) {
                    if (Double.isNaN(inBandSince)) {
                        inBandSince = now;
                    }
                    if (now - inBandSince >= RECOVER_HOLD) {
                        recoverTime = inBandSince - dipStart;
                        peakSpeed = speed;
                        setPhase(Phase.TAIL);
                    }
                } else {
                    inBandSince = Double.NaN;
                }
                if (phase == Phase.RECOVERING && now - dipStart >= RECOVER_TIMEOUT) {
                    recoverTime = RECOVER_TIMEOUT;
                    peakSpeed = sp;
                    finishShot("did not recover within " + (int) RECOVER_TIMEOUT + " s");
                }
                break;

            case TAIL:
                peakSpeed = Math.max(peakSpeed, speed);
                if (elapsed >= OVERSHOOT_WINDOW) {
                    finishShot("recovered");
                }
                break;

            case DONE:
                break;
        }
    }

    private void finishShot(String result) {
        double sp = Math.abs(setpoint);
        double dip = Math.max(0, (sp - minSpeed) / sp);
        double overshoot = Math.max(0, (peakSpeed - sp) / sp);
        worstRecovery = Math.max(worstRecovery, recoverTime);
        shots++;

        SmartDashboard.putNumber(name + "/Last_DipDepth", dip);
        SmartDashboard.putNumber(name + "/Last_RecoveryTime", recoverTime);
        SmartDashboard.putNumber(name + "/Last_Overshoot", overshoot);
        SmartDashboard.putNumber(name + "/Worst_RecoveryTime", worstRecovery);
        publishCounts();

        String summary = String.format("Shot %d: dip %.1f%%, %s in %.2f s, overshoot %.1f%%",
                shots, dip * 100, result, recoverTime, overshoot * 100);
        if (shots >= totalShots) {
            status("COMPLETE — " + summary + " (worst recovery " + String.format("%.2f", worstRecovery) + " s)");
            phase = Phase.DONE;
        } else {
            setPhase(Phase.ARMED);
            status(summary + " — load again (shot " + (shots + 1) + " of " + totalShots + ")");
        }
    }

    @Override
    public boolean isFinished() {
        return phase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        target.setSetpoint(0);
        target.resetIntegral();
        if (interrupted) {
            status("Cancelled after " + shots + " shot(s)");
        }
    }

    boolean failed() {
        return failed;
    }

    private void publishCounts() {
        SmartDashboard.putNumber(name + "/Shots", shots);
    }

    private void setPhase(Phase next) {
        phase = next;
        phaseStart = Timer.getFPGATimestamp();
    }

    private void status(String msg) {
        SmartDashboard.putString(name + "/Status", msg);
    }
}
