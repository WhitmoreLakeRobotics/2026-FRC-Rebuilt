package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Runs AutoTuneRecovery against the simulated roller starting from deliberately weak gains. */
class AutoTuneRecoveryTest {

    private static final double SP = 4800;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
    }

    private String key(String k) {
        return SmartDashboard.getString("FakeRoller/AutoTuneRecovery/" + k, "");
    }

    private double num(String k) {
        return SmartDashboard.getNumber("FakeRoller/AutoTuneRecovery/" + k, -1);
    }

    private AutoTuneVelocityTest.FakeRoller roller(double kP, double kI, double kV) {
        var r = new AutoTuneVelocityTest.FakeRoller(1.0, kV);
        r.kP = kP;
        r.kI = kI;
        return r;
    }

    private double run(AutoTuneVelocityTest.FakeRoller r, AutoTuneRecovery cmd, double maxSeconds) {
        cmd.initialize();
        double t = 0;
        while (!cmd.isFinished() && t < maxSeconds) {
            r.step();
            SimHooks.stepTiming(0.02);
            cmd.execute();
            t += 0.02;
        }
        cmd.end(false);
        System.out.printf("  finished in %.0f s, status: %s%n", t, key("Status"));
        return t;
    }

    /** Seconds after a 15% impulse until the speed is back inside 3% for good, with the roller's gains. */
    private double impulseRecovery(AutoTuneVelocityTest.FakeRoller r) {
        r.setSetpoint(SP);
        r.accum = 0;
        for (int i = 0; i < 250; i++) r.step();                    // 5 s to settle
        r.omega -= 0.15 * SP;
        double last = 0;
        for (int i = 1; i <= 100; i++) {                           // 2 s
            r.step();
            if (Math.abs(r.measured - SP) > 0.03 * SP) last = i * 0.02;
        }
        return last;
    }

    @Test
    void improvesRecoveryFromWeakGains() {
        var r = roller(0.0002, 0.0, 0.002072);
        double before = impulseRecovery(roller(0.0002, 0.0, 0.002072));

        run(r, new AutoTuneRecovery(r, r, SP), 1500);
        double after = impulseRecovery(r);
        System.out.printf("  kP=%.6f kI=%.7f | impulse recovery %.2f s -> %.2f s%n", r.kP, r.kI, before, after);

        assertTrue(key("Status").startsWith("COMPLETE"), key("Status"));
        assertTrue(num("Best_Score") < num("Baseline_Score"), "score should improve");
        assertTrue(after < before, "recovery " + after + " should beat " + before);
        assertTrue(r.kP > 0.0002, "kP should rise from the weak start");
        assertEquals(0.002072, r.kV, 1e-12, "kV must not change");
        assertTrue(num("Evaluations") <= AutoTuneRecovery.MAX_EVALS);
    }

    @Test
    void refusesToRunWithoutStartingGains() {
        var r = roller(0, 0, 0);
        run(r, new AutoTuneRecovery(r, r, SP), 10);
        assertTrue(key("Status").startsWith("FAILED"), key("Status"));
    }

    @Test
    void restoresGainsWhenInterrupted() {
        var r = roller(0.0002, 0.00002, 0.002072);
        AutoTuneRecovery cmd = new AutoTuneRecovery(r, r, SP);
        cmd.initialize();
        for (int i = 0; i < 700; i++) {                            // 14 s: mid-search
            r.step();
            SimHooks.stepTiming(0.02);
            cmd.execute();
        }
        cmd.end(true);
        assertEquals(0.0002, r.kP, 1e-12);
        assertEquals(0.00002, r.kI, 1e-12);
        assertEquals(0.002072, r.kV, 1e-12);
    }
}
