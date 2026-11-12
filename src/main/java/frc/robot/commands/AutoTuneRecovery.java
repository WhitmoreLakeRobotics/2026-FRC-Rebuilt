package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PIDTunable;

/**
 * AutoTuneRecovery — Part 2 of auto tuning: optimizes kP and kI for recovering from a load.
 *
 * Run AutoTuneVelocity first. It finds a working kV, kP and kI; this command starts from the gains
 * on the controller and searches for a kP/kI pair that recovers from a load faster.
 *
 * The load: the mechanism runs at the setpoint, then kV is cut by LOAD_FRACTION for PULSE_TIME and
 * restored. Cutting kV is the same as the mechanism suddenly needing extra drive, so the speed sags and
 * the feedback (kP, kI) has to bring it back, like a game piece hitting the roller. It also winds up the
 * integrator the way a real load does, so excess kI shows up as overshoot after the load is gone.
 *
 * The score of a gain pair is the average speed error (fraction of setpoint) over WINDOW seconds
 * starting at the load, plus OVERSHOOT_WEIGHT * overshoot, averaged over REPS loads. Lower is better.
 * A pair is rejected outright if it overshoots more than MAX_OVERSHOOT or makes the speed noisy.
 *
 * The search is a compass search on kP and kI. Each round it tries kP and kI times and divided by a
 * step factor, keeps any pair that scores better by IMPROVE_MARGIN, and shrinks the step when a
 * whole round finds nothing. It ends when the step is small or MAX_EVALS pairs have been tried.
 * kP and kI are limited to a range around the starting gains, and kD and kV are never changed.
 *
 * The final gains stay on the controller (not saved to flash). If the command is interrupted or
 * fails, the original gains are restored. Check the result with RecoveryTest and a real load.
 *
 * SAFETY: the mechanism runs at the setpoint for several minutes and its speed dips on purpose.
 *
 * Usage in RobotContainer:
 *   SmartDashboard.putData("AutoTune Intake Recovery",
 *       new AutoTuneRecovery(m_intake, m_intake, 4800));
 *
 * SmartDashboard keys (e.g. "Intake/AutoTuneRecovery/..."):
 *   Status, Recommended_kP, Recommended_kI, Summary, Baseline_Score, Best_Score,
 *   Baseline_RecoveryTime, Best_RecoveryTime, Trial_kP, Trial_kI, Trial_Score, Evaluations
 */
public class AutoTuneRecovery extends Command {

    /** kV is multiplied by (1 - this) during the load. */
    static final double LOAD_FRACTION = 0.25;
    static final double PULSE_TIME = 0.3;
    /** How long each load is scored for, counted from the start of the load. */
    static final double WINDOW = 1.5;
    static final int    REPS = 2;
    static final double SPINUP_TIME = 3.0;
    static final double SETTLE_TIME = 1.5;
    static final double SETTLE_TIME_REPEAT = 1.0;
    static final double RIPPLE_WINDOW = 0.5;

    static final double OVERSHOOT_WEIGHT = 0.5;
    static final double MAX_OVERSHOOT = 0.10;
    static final double MIN_RIPPLE = 0.03;
    static final double RIPPLE_NOISE_MULT = 2.0;
    /** Within this fraction of the setpoint counts as recovered. */
    static final double RECOVER_BAND = 0.03;

    static final double START_STEP = 2.0;
    static final double MIN_STEP = 1.15;
    static final int    MAX_EVALS = 36;
    /** A pair must beat the best score by this fraction to be accepted, so noise is not chased. */
    static final double IMPROVE_MARGIN = 0.03;
    static final double KP_MAX_FACTOR = 8.0;
    static final double KP_MIN_FACTOR = 1.0 / 16.0;
    static final double KI_MAX_FACTOR = 20.0;
    /** If kI starts at 0, its first trial is this fraction of kP. */
    static final double KI_SEED_FRACTION = 0.1;

    private enum Phase { SPINUP, SETTLE, PULSE, MEASURE, DONE }

    private final PIDTunable target;
    private final String name;
    private final double setpoint;
    private final double dir;

    private Phase phase;
    private double phaseStart;
    private double lastTime;
    private double[] originalGains;
    private boolean succeeded;

    private double kD, kV;
    private double startKP, startKI;

    // Search state
    private double bestKP, bestKI, bestScore = Double.POSITIVE_INFINITY, bestRecovery;
    private double baselineScore, baselineRecovery;
    private double curKP, curKI;
    private double step;
    private int candidate;
    private boolean improvedThisRound;
    private boolean evaluatingBaseline;
    private int evals;
    private double rippleLimit;

    // One pair's evaluation
    private int rep;
    private double scoreSum, overshootSum, recoverySum, rippleMax;

    // One load
    private double pulseStart, iae, peak, lastOutOfBand;
    private double min, max;
    private int count;

    public AutoTuneRecovery(PIDTunable target, SubsystemBase subsystem, double setpoint) {
        this.target   = target;
        this.name     = target.getTunableName() + "/AutoTuneRecovery";
        this.setpoint = setpoint;
        this.dir      = Math.signum(setpoint);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        originalGains = target.getPID();
        succeeded = false;
        kD = originalGains[2];
        kV = originalGains[3];
        startKP = originalGains[0];
        startKI = originalGains[1];

        if (kV <= 0 || startKP <= 0) {
            status("FAILED: run AutoTuneVelocity first — need a nonzero kV and kP to start from");
            phase = Phase.DONE;
            return;
        }

        bestKP = startKP;
        bestKI = startKI;
        step = START_STEP;
        candidate = 0;
        evals = 0;
        improvedThisRound = false;
        evaluatingBaseline = true;
        bestScore = Double.POSITIVE_INFINITY;

        target.resetIntegral();
        target.setSetpoint(setpoint);
        lastTime = Timer.getFPGATimestamp();
        setPhase(Phase.SPINUP);
        status("Spinning up to " + String.format("%.0f", setpoint));
    }

    @Override
    public void execute() {
        if (phase == Phase.DONE) {
            return;
        }
        double now     = Timer.getFPGATimestamp();
        double dt      = now - lastTime;
        lastTime       = now;
        double elapsed = now - phaseStart;
        double meas    = target.getMeasurement();
        double speed   = meas * dir;
        double sp      = Math.abs(setpoint);

        SmartDashboard.putNumber(name + "/Measurement", meas);
        target.setSetpoint(setpoint);

        switch (phase) {

            case SPINUP:
                if (elapsed >= SPINUP_TIME) {
                    startEvaluation(startKP, startKI);
                }
                break;

            case SETTLE:
                double settle = rep == 0 ? SETTLE_TIME : SETTLE_TIME_REPEAT;
                if (elapsed >= settle - RIPPLE_WINDOW) {
                    addSample(speed);
                }
                if (elapsed >= settle) {
                    rippleMax = Math.max(rippleMax, (max - min) / sp);
                    beginLoad(now);
                }
                break;

            case PULSE:
                trackLoad(speed, sp, dt, now, false);
                if (now - pulseStart >= PULSE_TIME) {
                    target.applyPID(curKP, curKI, kD, kV);
                    setPhase(Phase.MEASURE);
                }
                break;

            case MEASURE:
                trackLoad(speed, sp, dt, now, true);
                if (now - pulseStart >= WINDOW) {
                    finishLoad(sp);
                }
                break;

            case DONE:
                break;
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
        if (!succeeded) {
            target.applyPID(originalGains[0], originalGains[1], originalGains[2], originalGains[3]);
            if (interrupted) {
                status("Interrupted — original gains restored");
            }
        }
    }

    // -----------------------------------------------------------------------
    // One evaluation = REPS loads with one kP/kI pair
    // -----------------------------------------------------------------------

    private void startEvaluation(double p, double i) {
        curKP = p;
        curKI = i;
        rep = 0;
        scoreSum = overshootSum = recoverySum = rippleMax = 0;
        SmartDashboard.putNumber(name + "/Trial_kP", curKP);
        SmartDashboard.putNumber(name + "/Trial_kI", curKI);
        target.applyPID(curKP, curKI, kD, kV);
        startSettle();
    }

    private void startSettle() {
        target.resetIntegral();
        resetSamples();
        setPhase(Phase.SETTLE);
    }

    private void beginLoad(double now) {
        pulseStart = now;
        iae = 0;
        peak = Double.NEGATIVE_INFINITY;
        lastOutOfBand = 0;
        target.applyPID(curKP, curKI, kD, kV * (1 - LOAD_FRACTION));
        setPhase(Phase.PULSE);
    }

    private void trackLoad(double speed, double sp, double dt, double now, boolean afterRelease) {
        iae += Math.abs(sp - speed) * dt;
        if (afterRelease) {
            peak = Math.max(peak, speed);
        }
        if (Math.abs(speed - sp) > RECOVER_BAND * sp) {
            lastOutOfBand = now - pulseStart;
        }
    }

    private void finishLoad(double sp) {
        double overshoot = Math.max(0, (peak - sp) / sp);
        scoreSum     += iae / (sp * WINDOW) + OVERSHOOT_WEIGHT * overshoot;
        overshootSum += overshoot;
        recoverySum  += lastOutOfBand;
        rep++;
        if (rep < REPS) {
            startSettle();
        } else {
            finishEvaluation();
        }
    }

    private void finishEvaluation() {
        evals++;
        double overshoot = overshootSum / REPS;
        double score     = scoreSum / REPS;
        double recovery  = recoverySum / REPS;
        SmartDashboard.putNumber(name + "/Evaluations", evals);

        if (evaluatingBaseline) {
            evaluatingBaseline = false;
            rippleLimit = Math.max(MIN_RIPPLE, RIPPLE_NOISE_MULT * rippleMax);
            baselineScore = score;
            baselineRecovery = recovery;
            bestScore = score;
            bestRecovery = recovery;
            SmartDashboard.putNumber(name + "/Baseline_Score", score);
            SmartDashboard.putNumber(name + "/Baseline_RecoveryTime", recovery);
            status("Baseline score " + String.format("%.4f", score) + ", recovery "
                    + String.format("%.2f", recovery) + " s — searching");
            nextCandidate();
            return;
        }

        boolean bad = overshoot > MAX_OVERSHOOT || rippleMax > rippleLimit;
        SmartDashboard.putNumber(name + "/Trial_Score", bad ? -1 : score);

        if (!bad && score < bestScore * (1 - IMPROVE_MARGIN)) {
            bestScore = score;
            bestRecovery = recovery;
            bestKP = curKP;
            bestKI = curKI;
            improvedThisRound = true;
            SmartDashboard.putNumber(name + "/Best_Score", bestScore);
            SmartDashboard.putNumber(name + "/Best_RecoveryTime", bestRecovery);
            status("Better: kP " + String.format("%.6f", bestKP) + " kI " + String.format("%.7f", bestKI)
                    + " score " + String.format("%.4f", bestScore));
        }
        nextCandidate();
    }

    /** Picks the next kP/kI pair to try from the best so far, or finishes the search. */
    private void nextCandidate() {
        while (evals < MAX_EVALS) {
            if (candidate >= 4) {
                if (!improvedThisRound) {
                    step = Math.sqrt(step);
                    if (step < MIN_STEP) {
                        break;
                    }
                }
                candidate = 0;
                improvedThisRound = false;
            }
            double p = bestKP;
            double i = bestKI;
            switch (candidate) {
                case 0: p = bestKP * step; break;
                case 1: p = bestKP / step; break;
                case 2: i = bestKI > 0 ? bestKI * step : bestKP * KI_SEED_FRACTION; break;
                default: i = bestKI > 0 ? bestKI / step : -1; break;
            }
            candidate++;

            boolean inRange = p >= startKP * KP_MIN_FACTOR && p <= startKP * KP_MAX_FACTOR
                    && i >= 0 && i <= Math.max(startKI, startKP * KI_SEED_FRACTION) * KI_MAX_FACTOR;
            if (inRange) {
                startEvaluation(p, i);
                return;
            }
        }
        complete();
    }

    private void complete() {
        succeeded = true;
        target.applyPID(bestKP, bestKI, kD, kV);
        SmartDashboard.putNumber(name + "/Recommended_kP", bestKP);
        SmartDashboard.putNumber(name + "/Recommended_kI", bestKI);
        SmartDashboard.putNumber(name + "/Best_Score", bestScore);
        SmartDashboard.putNumber(name + "/Best_RecoveryTime", bestRecovery);
        SmartDashboard.putString(name + "/Summary",
                String.format("kP=%.6f kI=%.7f kD=%.7f kV=%.6f @ %.0f", bestKP, bestKI, kD, kV, setpoint));
        boolean better = bestScore < baselineScore;
        status((better ? "COMPLETE" : "COMPLETE — no better gains found, kept the starting gains")
                + " (score " + String.format("%.4f", baselineScore) + " -> " + String.format("%.4f", bestScore)
                + ", recovery " + String.format("%.2f", baselineRecovery) + " s -> "
                + String.format("%.2f", bestRecovery) + " s, " + evals + " trials)");
        phase = Phase.DONE;
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    private void setPhase(Phase next) {
        phase = next;
        phaseStart = Timer.getFPGATimestamp();
    }

    private void resetSamples() {
        count = 0;
        min = Double.POSITIVE_INFINITY;
        max = Double.NEGATIVE_INFINITY;
    }

    private void addSample(double value) {
        count++;
        min = Math.min(min, value);
        max = Math.max(max, value);
    }

    private void status(String msg) {
        SmartDashboard.putString(name + "/Status", msg);
    }
}
