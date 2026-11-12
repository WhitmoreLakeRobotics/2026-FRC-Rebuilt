package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PIDTunable;
import java.util.Random;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Runs AutoTuneVelocity against a simulated velocity-controlled motor
 * (first-order lag, voltage limit, static friction, filtered noisy sensor).
 */
class AutoTuneVelocityTest {

    /** Simulated SparkMax velocity loop. `unit` scales how the controller turns gains into volts. */
    static class FakeRoller extends SubsystemBase implements PIDTunable {
        final double unit;
        final double rpmPerVolt = 6000.0 / 12.0;
        final double tau = 0.25;
        final double kS = 0.3;
        final double maxVolts = 12.0;
        final Random rng = new Random(1);

        double omega = 0;
        double kP, kI, kD, kV;
        double setpoint = 0;
        double accum = 0;
        double prevErr = 0;
        final double[] filt;
        int filtIdx = 0;
        double measured = 0;

        FakeRoller(double unit, double startKv) {
            this(unit, startKv, 20);
        }

        /** @param filterTaps ms of sensor averaging; a large value adds lag that can make the loop overshoot */
        FakeRoller(double unit, double startKv, int filterTaps) {
            this.unit = unit;
            this.kV = startKv;
            this.filt = new double[filterTaps];
        }

        /** Advance physics by one 20 ms robot loop (1 ms controller ticks). */
        void step() {
            for (int i = 0; i < 20; i++) {
                double err = setpoint - measured;
                accum += err * 0.001;
                double d = (err - prevErr) / 0.001;
                prevErr = err;
                double out = unit * (kV * setpoint + kP * err + kI * accum + kD * d);
                double volts = Math.max(-maxVolts, Math.min(maxVolts, out));
                double eff = Math.abs(volts) > kS ? volts - Math.signum(volts) * kS : 0;
                omega += (eff * rpmPerVolt - omega) / tau * 0.001;
                filt[filtIdx] = omega + rng.nextGaussian() * 8;
                filtIdx = (filtIdx + 1) % filt.length;
                double s = 0;
                for (double v : filt) s += v;
                measured = s / filt.length;
            }
        }

        @Override public double getMeasurement() { return measured; }
        @Override public void setSetpoint(double sp) { setpoint = sp; }
        @Override public void applyPID(double p, double i, double d, double v) { kP = p; kI = i; kD = d; kV = v; }
        @Override public void resetIntegral() { accum = 0; }
        @Override public double[] getPID() { return new double[] { kP, kI, kD, kV }; }
        @Override public String getTunableName() { return "FakeRoller"; }
    }

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
    }

    private String key(String k) {
        return SmartDashboard.getString("FakeRoller/AutoTune/" + k, "");
    }

    private double num(String k) {
        return SmartDashboard.getNumber("FakeRoller/AutoTune/" + k, -1);
    }

    private double runCommand(FakeRoller roller, AutoTuneVelocity cmd, double maxSeconds) {
        cmd.initialize();
        double t = 0;
        while (!cmd.isFinished() && t < maxSeconds) {
            roller.step();
            SimHooks.stepTiming(0.02);
            cmd.execute();
            t += 0.02;
        }
        cmd.end(false);
        System.out.printf("  finished in %.0f s, status: %s%n", t, key("Status"));
        return t;
    }

    private void printResult(String label, FakeRoller r) {
        System.out.printf("%s -> kP=%.6f kI=%.7f kD=%.7f kV=%.6f | verify overshoot=%.3f ripple=%.3f loadErr=%.3f%n",
                label, r.kP, r.kI, r.kD, r.kV,
                num("Verify_Overshoot"), num("Verify_Ripple"), num("Verify_LoadError"));
    }

    /** Settled error (fraction of setpoint) with the given gains and kV scaled to fake a load. */
    private double settledError(FakeRoller r, double sp, double kvScale) {
        double kv = r.kV;
        r.kV = kv * kvScale;
        r.accum = 0;
        r.setSetpoint(sp);
        for (int i = 0; i < 400; i++) r.step();     // 8 s
        double err = Math.abs(r.measured - sp) / Math.abs(sp);
        r.kV = kv;
        return err;
    }

    @ParameterizedTest
    @ValueSource(doubles = { 1.0, 0.1, 10.0 })
    void findsAllGains(double unit) {
        FakeRoller roller = new FakeRoller(unit, 0.0018);
        AutoTuneVelocity cmd = new AutoTuneVelocity(roller, roller, 4800);

        runCommand(roller, cmd, 600);
        printResult("unit=" + unit, roller);

        assertTrue(key("Status").startsWith("COMPLETE"), key("Status"));
        assertTrue(roller.kP > 0, "kP should be found");
        assertTrue(roller.kI > 0, "kI should be found, a 15% load leaves error with kP alone");

        // feedforward alone reaches the setpoint
        double noGains = settledError(new FakeRoller(unit, roller.kV) {{ }}, 4800, 1.0);
        assertTrue(noGains < 0.05);

        // with the load, the tuned gains recover close to the setpoint
        assertTrue(settledError(roller, 4800, 0.85) < 0.03, "tuned gains should handle a load");
    }

    @ParameterizedTest
    @ValueSource(doubles = { 4800.0, -4257.0 })
    void handlesDirection(double sp) {
        FakeRoller roller = new FakeRoller(1.0, 0.0018);
        AutoTuneVelocity cmd = new AutoTuneVelocity(roller, roller, sp);
        runCommand(roller, cmd, 600);
        printResult("sp=" + sp, roller);
        assertTrue(key("Status").startsWith("COMPLETE"), key("Status"));
        assertTrue(roller.kI > 0);
        assertTrue(settledError(roller, sp, 0.85) < 0.03);
    }

    @Test
    void laggySensorStillFinishesWithSafeGains() {
        FakeRoller roller = new FakeRoller(1.0, 0.0018, 120);
        AutoTuneVelocity cmd = new AutoTuneVelocity(roller, roller, 4800);
        runCommand(roller, cmd, 900);
        printResult("laggy", roller);
        assertTrue(key("Status").startsWith("COMPLETE"), key("Status"));
        assertTrue(roller.kP >= 0 && roller.kI >= 0 && roller.kD >= 0);
        assertTrue(num("Verify_Overshoot") < 0.15, "verify overshoot " + num("Verify_Overshoot"));
    }

    @Test
    void restoresGainsWhenMotorDoesNotSpin() {
        FakeRoller roller = new FakeRoller(0.0, 0.0018); // unit 0 = motor never moves
        roller.kP = 0.0003;
        roller.kI = 0.00002;
        AutoTuneVelocity cmd = new AutoTuneVelocity(roller, roller, 4800);
        runCommand(roller, cmd, 60);
        assertTrue(key("Status").startsWith("FAILED"));
        assertEquals(0.0003, roller.kP, 1e-12);
        assertEquals(0.00002, roller.kI, 1e-12);
        assertEquals(0.0018, roller.kV, 1e-12);
    }
}
