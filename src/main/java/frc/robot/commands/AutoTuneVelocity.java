package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PIDTunable;

/**
 * AutoTuneVelocity — Automatic velocity PIDF tuning for one motor
 *
 * Finds kV, kP, kI and kD for a velocity loop running on the motor controller
 * (slot 0) by measuring the real mechanism, so you do not have to guess the gains.
 *
 * Step 1 — kV (feedforward):
 *   Runs the motor at the tuning setpoint with kP = 0 and measures the speed it
 *   actually reaches. Scales kV by (setpoint / measured) and repeats until the
 *   speed is within KV_TOLERANCE of the setpoint. This is self-calibrating, so it
 *   does not depend on what units the controller uses for kV.
 *
 * Step 2 — kP (feedback):
 *   Steps from rest to the setpoint with a small kP, doubling it each trial and
 *   timing how long the speed takes to reach 90%. A trial passes if overshoot and
 *   ripple stay small. The search stops at the first of:
 *     - the knee: doubling kP no longer speeds up the response -> keep the lower kP
 *     - instability: a trial overshoots or ripples -> KP_SAFETY * last passing kP
 *     - the trial limit -> KP_SAFETY * last passing kP
 *
 * Step 3 — kI (removes the error left by load):
 *   There is no way to apply a real load, so the test fakes one: kV is set
 *   DISTURBANCE_KV_FRACTION of its true value, which is the same as the mechanism
 *   suddenly needing extra drive (a heavy load, battery sag). kP alone leaves a
 *   steady error. kI is raised (x KI_FACTOR per trial) until the speed ends up
 *   within KI_TARGET_ERROR of the setpoint by the end of the step, or a trial
 *   overshoots. If kP alone already meets the target, kI stays 0.
 *
 * Step 4 — kD (damping):
 *   A velocity loop rarely needs kD and it amplifies sensor noise, so it is only
 *   tuned if the step with the real kV still overshoots more than KD_NEEDED_OVERSHOOT.
 *   Then kD is raised until the overshoot is damped or ripple gets too noisy, and
 *   it is only kept if it reduced the overshoot noticeably. Otherwise kD stays 0.
 *
 * Step 5 — verify:
 *   A step with the final gains and the real kV (overshoot/ripple), then, if kI is
 *   used, a step with the fake load (remaining error). Both are published.
 *
 * The final gains stay on the controller (not saved to flash) so you can test them
 * right away. Copy them into the subsystem's motor config to keep them.
 * If the command is interrupted or fails, the original gains are restored.
 *
 * SAFETY: the motor spins up to the tuning setpoint repeatedly, from rest, for
 * a few minutes. Make sure the mechanism is free to spin.
 *
 * Usage in RobotContainer:
 *   SmartDashboard.putData("AutoTune Intake",
 *       new AutoTuneVelocity(m_intake, m_intake, 4800));
 *
 * SmartDashboard keys (e.g. "Intake/AutoTune/..."):
 *   Status, Recommended_kP, Recommended_kI, Recommended_kD, Recommended_kV, Summary,
 *   Trial_kP, Trial_kI, Trial_kD, Trial_Overshoot, Trial_Ripple, Trial_RiseTime,
 *   Trial_SteadyError, Verify_Overshoot, Verify_Ripple, Verify_LoadError
 */
public class AutoTuneVelocity extends Command {

    // -----------------------------------------------------------------------
    // Tuning parameters
    // -----------------------------------------------------------------------

    /** Starting kV if the controller reports 0. */
    private static final double KV_START = 0.002;
    private static final int    KV_MAX_ITERATIONS = 12;
    /** kV is treated as saturated if speed changed by less than this fraction of what proportional scaling predicts. */
    private static final double SATURATION_SLOPE_FRACTION = 0.25;
    /** kV is accepted when the measured speed is within this fraction of the setpoint. */
    private static final double KV_TOLERANCE = 0.01;
    private static final double KV_FIRST_SETTLE_TIME = 3.0;
    private static final double KV_SETTLE_TIME = 1.5;
    private static final double KV_SAMPLE_TIME = 1.0;

    private static final double KP_START = 0.00001;
    private static final double KP_FACTOR = 2.0;
    private static final int    KP_MAX_TRIALS = 12;
    /** When a search ends on instability or its limit, the result = this * the last value that passed. */
    private static final double SAFETY = 0.8;
    /** Rise time is measured to this fraction of the setpoint. */
    private static final double RISE_FRACTION = 0.9;
    /** The knee is reached when doubling kP improves rise time by less than this fraction... */
    private static final double KP_KNEE_IMPROVEMENT = 0.10;
    /** ...but only once rise time has improved by at least this fraction over the smallest kP. */
    private static final double KP_MIN_TOTAL_IMPROVEMENT = 0.20;

    /** kV is multiplied by this to fake a load while tuning kI. */
    private static final double DISTURBANCE_KV_FRACTION = 0.85;
    /** kI trials start at this fraction of kP (unit-agnostic: the search climbs from here). */
    private static final double KI_START_FRACTION = 0.0001;
    private static final double KI_FACTOR = 3.0;
    private static final int    KI_MAX_TRIALS = 12;
    /** kI is good enough when the speed is within this fraction of the setpoint by the end of the step. */
    private static final double KI_TARGET_ERROR = 0.015;
    /**
     * kI is tuned with a fake load, but without a load the integral winds up during spin-up and
     * overshoots more. If the final clean verification overshoots more than MAX_OVERSHOOT, kI is
     * multiplied by this and verified again, up to KI_BACKOFF_MAX times.
     */
    private static final double KI_BACKOFF = 0.7;
    private static final int    KI_BACKOFF_MAX = 4;
    private static final double KI_STEP_TIME = 3.0;
    private static final double KI_FINAL_WINDOW = 0.75;

    /** kD is only tuned if the clean step overshoots more than this. */
    private static final double KD_NEEDED_OVERSHOOT = 0.05;
    /** kD search stops once overshoot is this low. */
    private static final double KD_TARGET_OVERSHOOT = 0.03;
    /** kD is only kept if it brought overshoot down to this fraction of the no-kD overshoot. */
    private static final double KD_MIN_REDUCTION = 0.8;
    private static final double KD_START_FRACTION = 0.00001;
    private static final double KD_FACTOR = 3.0;
    private static final int    KD_MAX_TRIALS = 9;

    private static final double STEP_TIME = 2.0;
    /** The last part of each step used to measure ripple. */
    private static final double FINAL_WINDOW = 0.5;
    private static final double SPINDOWN_TIMEOUT = 3.0;
    private static final double SPINDOWN_FRACTION = 0.05;

    private static final double MAX_OVERSHOOT = 0.10;
    /** Minimum ripple allowed; also never less than RIPPLE_NOISE_MULT * the ripple measured in step 1. */
    private static final double MIN_RIPPLE = 0.03;
    private static final double RIPPLE_NOISE_MULT = 2.0;

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------

    private enum Phase { KV_SETTLE, KV_SAMPLE, SPINDOWN, STEP, DONE }

    /** What the step currently being run is for. */
    private enum Trial { KP, KI, KD_CHECK, KD, VERIFY, VERIFY_LOAD }

    private final PIDTunable target;
    private final String name;
    private final double setpoint;
    private final double dir;

    private Phase phase;
    private Trial trial;
    private double phaseStart;
    private double[] originalGains;
    private boolean succeeded;

    // Step 1
    private double kV;
    private int kvIteration;
    private double prevKv, prevMean;
    private double noiseRipple;     // ripple (fraction of setpoint) with kP = 0

    // Step 2
    private double kP;
    private int kpTrial;
    private double lastStableKP;
    private double baselineRise, prevRise;

    // Step 3
    private double kI;
    private int kiTrial;
    private double lastStableKI;

    // Step 4
    private double kD;
    private int kdTrial;
    private double bestKD, bestOvershoot, noKdOvershoot;

    // Step 5
    private int kiBackoffs;
    private String verifyMsg = "COMPLETE";

    // Timing of the step currently running
    private double trialStepTime = STEP_TIME;
    private double trialWindow   = FINAL_WINDOW;

    // Per-phase sampling
    private double sum, min, max, peak, riseTime;
    private int count;

    /**
     * @param target    The subsystem to tune (must implement PIDTunable)
     * @param subsystem The same subsystem cast as SubsystemBase (for requirements)
     * @param setpoint  The velocity to tune at (e.g. RPM); use the speed the mechanism normally runs at
     */
    public AutoTuneVelocity(PIDTunable target, SubsystemBase subsystem, double setpoint) {
        this.target   = target;
        this.name     = target.getTunableName() + "/AutoTune";
        this.setpoint = setpoint;
        this.dir      = Math.signum(setpoint);
        addRequirements(subsystem);
    }

    // -----------------------------------------------------------------------
    // Command lifecycle
    // -----------------------------------------------------------------------

    @Override
    public void initialize() {
        originalGains = target.getPID();
        succeeded = false;

        kV = originalGains[3] > 0 ? originalGains[3] : KV_START;
        kvIteration = 0;
        lastStableKP = 0;
        kpTrial = 0;
        kP = KP_START;
        kI = 0;
        kD = 0;
        kiBackoffs = 0;

        target.resetIntegral();
        target.applyPID(0, 0, 0, kV);
        target.setSetpoint(setpoint);
        setPhase(Phase.KV_SETTLE);
        status("Step 1: finding kV, iteration 1");
    }

    @Override
    public void execute() {
        double now     = Timer.getFPGATimestamp();
        double elapsed = now - phaseStart;
        double meas    = target.getMeasurement();
        double dirMeas = meas * dir;

        SmartDashboard.putNumber(name + "/Measurement", meas);

        switch (phase) {

            case KV_SETTLE:
                target.setSetpoint(setpoint);
                if (elapsed >= (kvIteration == 0 ? KV_FIRST_SETTLE_TIME : KV_SETTLE_TIME)) {
                    resetStats();
                    setPhase(Phase.KV_SAMPLE);
                }
                break;

            case KV_SAMPLE:
                target.setSetpoint(setpoint);
                addSample(dirMeas);
                if (elapsed >= KV_SAMPLE_TIME) {
                    finishKvIteration();
                }
                break;

            case SPINDOWN:
                target.setSetpoint(0);
                if (Math.abs(meas) < SPINDOWN_FRACTION * Math.abs(setpoint) || elapsed >= SPINDOWN_TIMEOUT) {
                    // Braking against a setpoint of 0 winds the integrator up; start the step clean
                    target.resetIntegral();
                    resetStats();
                    setPhase(Phase.STEP);
                }
                break;

            case STEP:
                target.setSetpoint(setpoint);
                peak = Math.max(peak, dirMeas);
                if (Double.isNaN(riseTime) && dirMeas >= RISE_FRACTION * Math.abs(setpoint)) {
                    riseTime = elapsed;
                }
                if (elapsed >= trialStepTime - trialWindow) {
                    addSample(dirMeas);
                }
                if (elapsed >= trialStepTime) {
                    finishStep();
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
    // Step 1: kV
    // -----------------------------------------------------------------------

    private void finishKvIteration() {
        double mean = sum / count;
        double sp   = Math.abs(setpoint);

        if (mean < 0.02 * sp) {
            fail("Motor did not spin in the setpoint direction (measured " + String.format("%.0f", mean * dir)
                    + ") — check CAN ID, direction and that the mechanism is free");
            return;
        }

        SmartDashboard.putNumber(name + "/Trial_kV", kV);
        SmartDashboard.putNumber(name + "/Trial_kV_Measured", mean * dir);

        if (Math.abs(mean - sp) <= KV_TOLERANCE * sp) {
            noiseRipple = (max - min) / sp;
            status("kV = " + String.format("%.6f", kV) + " — step 2: finding kP");
            startTrial(Trial.KP, kP, 0, 0, kV, STEP_TIME, FINAL_WINDOW);
            return;
        }

        if (kvIteration + 1 >= KV_MAX_ITERATIONS) {
            fail("kV did not converge after " + KV_MAX_ITERATIONS + " iterations (last measured "
                    + String.format("%.0f", mean * dir) + " vs " + String.format("%.0f", setpoint)
                    + ") — the setpoint may be faster than the motor can reach");
            return;
        }

        // Speed should scale with kV. If it barely moved when kV changed, the output is
        // saturated, so scaling by the ratio would crawl — back off (or boost) geometrically.
        double factor = sp / mean;
        if (kvIteration > 0 && Math.abs(kV - prevKv) > 1e-12) {
            double observedSlope     = (mean - prevMean) / (kV - prevKv);
            double proportionalSlope = mean / kV;
            if (observedSlope < SATURATION_SLOPE_FRACTION * proportionalSlope) {
                factor = mean > sp ? 0.5 : 2.0;
            }
        }
        prevKv   = kV;
        prevMean = mean;
        kV *= factor;
        kvIteration++;
        target.applyPID(0, 0, 0, kV);
        setPhase(Phase.KV_SETTLE);
        status("Step 1: finding kV, iteration " + (kvIteration + 1));
    }

    // -----------------------------------------------------------------------
    // Steps 2-5: every trial is a step from rest, evaluated by what it is for
    // -----------------------------------------------------------------------

    /** Applies the gains for one trial and starts it (spin down, then step). */
    private void startTrial(Trial purpose, double p, double i, double d, double v,
                            double stepTime, double window) {
        trial = purpose;
        trialStepTime = stepTime;
        trialWindow = window;
        target.applyPID(p, i, d, v);
        setPhase(Phase.SPINDOWN);
    }

    private void finishStep() {
        double sp        = Math.abs(setpoint);
        double overshoot = Math.max(0, (peak - sp) / sp);
        double ripple    = (max - min) / sp;
        double steadyErr = Math.abs(sum / count - sp) / sp;
        double rise      = Double.isNaN(riseTime) ? trialStepTime : riseTime;
        boolean rippleOk = ripple <= Math.max(MIN_RIPPLE, RIPPLE_NOISE_MULT * noiseRipple);
        boolean stable   = overshoot <= MAX_OVERSHOOT && rippleOk;

        switch (trial) {
            case KP:          finishKpTrial(overshoot, ripple, rise, stable);          break;
            case KI:          finishKiTrial(overshoot, ripple, steadyErr, rippleOk);   break;
            case KD_CHECK:    finishKdCheck(overshoot);                                break;
            case KD:          finishKdTrial(overshoot, ripple, rippleOk);              break;
            case VERIFY:      finishVerify(overshoot, ripple, stable);                 break;
            case VERIFY_LOAD: finishVerifyLoad(steadyErr);                             break;
        }
    }

    // ---- Step 2: kP -------------------------------------------------------

    private void finishKpTrial(double overshoot, double ripple, double rise, boolean stable) {
        SmartDashboard.putNumber(name + "/Trial_kP", kP);
        SmartDashboard.putNumber(name + "/Trial_Overshoot", overshoot);
        SmartDashboard.putNumber(name + "/Trial_Ripple", ripple);
        SmartDashboard.putNumber(name + "/Trial_RiseTime", rise);

        if (!stable) {
            finishKpSearch(lastStableKP * SAFETY, "kP " + String.format("%.6f", kP) + " was unstable");
            return;
        }

        if (kpTrial == 0) {
            baselineRise = rise;
        } else if (baselineRise - rise >= KP_MIN_TOTAL_IMPROVEMENT * baselineRise
                && prevRise - rise < KP_KNEE_IMPROVEMENT * prevRise) {
            // Past the knee: doubling kP no longer speeds up the response, so keep the lower kP
            finishKpSearch(lastStableKP, "more kP no longer speeds up the response");
            return;
        }

        lastStableKP = kP;
        prevRise = rise;
        kpTrial++;
        if (kpTrial >= KP_MAX_TRIALS) {
            finishKpSearch(lastStableKP * SAFETY, "search limit reached without finding a knee or instability");
            return;
        }
        kP *= KP_FACTOR;
        status("Step 2: kP " + String.format("%.6f", lastStableKP) + " OK (rise "
                + String.format("%.2f", rise) + " s) — trying " + String.format("%.6f", kP));
        startTrial(Trial.KP, kP, 0, 0, kV, STEP_TIME, FINAL_WINDOW);
    }

    private void finishKpSearch(double chosenKP, String reason) {
        kP = chosenKP;

        if (kP == 0) {
            kI = 0;
            kD = 0;
            complete("COMPLETE — even the smallest kP trial was unstable, using feedforward only (kP = 0)");
            return;
        }

        status("kP = " + String.format("%.6f", kP) + " (" + reason + ") — step 3: finding kI");
        lastStableKI = 0;
        kiTrial = 0;
        kI = 0;
        startKiTrial();
    }

    // ---- Step 3: kI -------------------------------------------------------

    /** kI trials run with kV reduced so there is a steady error for kI to remove. */
    private void startKiTrial() {
        SmartDashboard.putNumber(name + "/Trial_kI", kI);
        startTrial(Trial.KI, kP, kI, 0, kV * DISTURBANCE_KV_FRACTION, KI_STEP_TIME, KI_FINAL_WINDOW);
    }

    private void finishKiTrial(double overshoot, double ripple, double steadyErr, boolean rippleOk) {
        SmartDashboard.putNumber(name + "/Trial_Overshoot", overshoot);
        SmartDashboard.putNumber(name + "/Trial_Ripple", ripple);
        SmartDashboard.putNumber(name + "/Trial_SteadyError", steadyErr);

        if (overshoot > MAX_OVERSHOOT || !rippleOk) {
            finishKiSearch(lastStableKI * SAFETY,
                    "kI " + String.format("%.7f", kI) + " overshoots or oscillates");
            return;
        }

        if (steadyErr <= KI_TARGET_ERROR) {
            finishKiSearch(kI, kI == 0
                    ? "kP alone recovers from a load, kI not needed"
                    : "recovers from a load to within " + String.format("%.1f%%", KI_TARGET_ERROR * 100));
            return;
        }

        lastStableKI = kI;
        kiTrial++;
        if (kiTrial >= KI_MAX_TRIALS) {
            finishKiSearch(lastStableKI * SAFETY, "search limit reached");
            return;
        }
        kI = (kI == 0) ? kP * KI_START_FRACTION : kI * KI_FACTOR;
        status("Step 3: load error " + String.format("%.1f%%", steadyErr * 100) + " — trying kI "
                + String.format("%.7f", kI));
        startKiTrial();
    }

    private void finishKiSearch(double chosenKI, String reason) {
        kI = chosenKI;
        status("kI = " + String.format("%.7f", kI) + " (" + reason + ") — step 4: checking kD");
        // Clean step (real kV) with kP and kI: does it overshoot enough to need damping?
        startTrial(Trial.KD_CHECK, kP, kI, 0, kV, STEP_TIME, FINAL_WINDOW);
    }

    // ---- Step 4: kD -------------------------------------------------------

    private void finishKdCheck(double overshoot) {
        noKdOvershoot = overshoot;
        SmartDashboard.putNumber(name + "/Trial_Overshoot", overshoot);

        if (overshoot <= KD_NEEDED_OVERSHOOT) {
            finishKdSearch("overshoot " + String.format("%.1f%%", overshoot * 100) + ", kD not needed");
            return;
        }

        kD = kP * KD_START_FRACTION;
        kdTrial = 0;
        bestKD = 0;
        bestOvershoot = overshoot;
        status("Step 4: overshoot " + String.format("%.1f%%", overshoot * 100) + " — trying kD "
                + String.format("%.7f", kD));
        startKdTrial();
    }

    private void startKdTrial() {
        SmartDashboard.putNumber(name + "/Trial_kD", kD);
        startTrial(Trial.KD, kP, kI, kD, kV, STEP_TIME, FINAL_WINDOW);
    }

    private void finishKdTrial(double overshoot, double ripple, boolean rippleOk) {
        SmartDashboard.putNumber(name + "/Trial_Overshoot", overshoot);
        SmartDashboard.putNumber(name + "/Trial_Ripple", ripple);

        if (!rippleOk) {
            finishKdSearch("kD " + String.format("%.7f", kD) + " made the speed too noisy");
            return;
        }
        if (overshoot < bestOvershoot) {
            bestOvershoot = overshoot;
            bestKD = kD;
        }
        kdTrial++;
        if (overshoot <= KD_TARGET_OVERSHOOT || kdTrial >= KD_MAX_TRIALS) {
            finishKdSearch(overshoot <= KD_TARGET_OVERSHOOT ? "overshoot damped" : "search limit reached");
            return;
        }
        kD *= KD_FACTOR;
        status("Step 4: overshoot " + String.format("%.1f%%", overshoot * 100) + " — trying kD "
                + String.format("%.7f", kD));
        startKdTrial();
    }

    private void finishKdSearch(String reason) {
        boolean helped = bestKD > 0 && bestOvershoot <= KD_MIN_REDUCTION * noKdOvershoot;
        kD = helped ? bestKD : 0;
        status("kD = " + String.format("%.7f", kD) + " (" + (helped || kdTrial == 0 ? reason : "kD did not help")
                + ") — step 5: verifying");
        startTrial(Trial.VERIFY, kP, kI, kD, kV, STEP_TIME, FINAL_WINDOW);
    }

    // ---- Step 5: verify ---------------------------------------------------

    private void finishVerify(double overshoot, double ripple, boolean stable) {
        SmartDashboard.putNumber(name + "/Verify_Overshoot", overshoot);
        SmartDashboard.putNumber(name + "/Verify_Ripple", ripple);

        if (overshoot > MAX_OVERSHOOT && kI > 0 && kiBackoffs < KI_BACKOFF_MAX) {
            kI *= KI_BACKOFF;
            kiBackoffs++;
            status("Step 5: overshoot " + String.format("%.1f%%", overshoot * 100)
                    + " without a load — lowering kI to " + String.format("%.7f", kI));
            startTrial(Trial.VERIFY, kP, kI, kD, kV, STEP_TIME, FINAL_WINDOW);
            return;
        }

        String msg = stable
                ? "COMPLETE"
                : "COMPLETE, but the verification step was not clean (overshoot "
                    + String.format("%.1f%%", overshoot * 100) + ", ripple "
                    + String.format("%.1f%%", ripple * 100) + ") — try lower gains in TunePID";

        if (kI > 0) {
            verifyMsg = msg;
            startTrial(Trial.VERIFY_LOAD, kP, kI, kD, kV * DISTURBANCE_KV_FRACTION, KI_STEP_TIME, KI_FINAL_WINDOW);
        } else {
            complete(msg);
        }
    }

    private void finishVerifyLoad(double steadyErr) {
        SmartDashboard.putNumber(name + "/Verify_LoadError", steadyErr);
        complete(verifyMsg);
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    private void complete(String statusMsg) {
        succeeded = true;
        target.applyPID(kP, kI, kD, kV);
        SmartDashboard.putNumber(name + "/Recommended_kP", kP);
        SmartDashboard.putNumber(name + "/Recommended_kI", kI);
        SmartDashboard.putNumber(name + "/Recommended_kD", kD);
        SmartDashboard.putNumber(name + "/Recommended_kV", kV);
        SmartDashboard.putString(name + "/Summary",
                String.format("kP=%.6f kI=%.7f kD=%.7f kV=%.6f @ %.0f", kP, kI, kD, kV, setpoint));
        status(statusMsg);
        phase = Phase.DONE;
    }

    private void fail(String msg) {
        status("FAILED: " + msg);
        phase = Phase.DONE;
    }

    private void setPhase(Phase next) {
        phase = next;
        phaseStart = Timer.getFPGATimestamp();
    }

    private void resetStats() {
        sum = 0;
        count = 0;
        min = Double.POSITIVE_INFINITY;
        max = Double.NEGATIVE_INFINITY;
        peak = Double.NEGATIVE_INFINITY;
        riseTime = Double.NaN;
    }

    private void addSample(double value) {
        sum += value;
        count++;
        min = Math.min(min, value);
        max = Math.max(max, value);
    }

    private void status(String msg) {
        SmartDashboard.putString(name + "/Status", msg);
    }
}
