package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Runs RecoveryTest against the simulated roller, loaded by an impulse that knocks the speed down. */
class RecoveryTestTest {

    private static final double SP = 4800;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
    }

    private String key(String k) {
        return SmartDashboard.getString("FakeRoller/Recovery/" + k, "");
    }

    private double num(String k) {
        return SmartDashboard.getNumber("FakeRoller/Recovery/" + k, -1);
    }

    /** Runs the command, hitting the roller with `dropFraction` of the setpoint at each time in `hitTimes`. */
    private void run(AutoTuneVelocityTest.FakeRoller roller, RecoveryTest cmd, double dropFraction,
                     double[] hitTimes, double maxSeconds) {
        cmd.initialize();
        double t = 0;
        int next = 0;
        while (!cmd.isFinished() && t < maxSeconds) {
            if (next < hitTimes.length && t >= hitTimes[next]) {
                roller.omega -= dropFraction * SP;
                next++;
            }
            roller.step();
            SimHooks.stepTiming(0.02);
            cmd.execute();
            t += 0.02;
        }
        cmd.end(false);
        System.out.printf("  t=%.0f s, status: %s%n", t, key("Status"));
    }

    private AutoTuneVelocityTest.FakeRoller roller(double kP, double kI) {
        var r = new AutoTuneVelocityTest.FakeRoller(1.0, 0.002072);
        r.kP = kP;
        r.kI = kI;
        return r;
    }

    @Test
    void measuresDipAndRecovery() {
        var r = roller(0.00128, 0.0017636);
        RecoveryTest cmd = new RecoveryTest(r, r, SP, 2);
        run(r, cmd, 0.15, new double[] { 6, 12 }, 60);

        assertTrue(key("Status").startsWith("COMPLETE"), key("Status"));
        assertTrue(num("Shots") == 2);
        assertTrue(num("Last_DipDepth") > 0.05, "dip " + num("Last_DipDepth"));
        assertTrue(num("Last_RecoveryTime") > 0 && num("Last_RecoveryTime") < 5, "recovery " + num("Last_RecoveryTime"));
        assertTrue(num("Worst_RecoveryTime") >= num("Last_RecoveryTime"));
    }

    @Test
    void higherGainsRecoverFaster() {
        var weak = roller(0.0002, 0.0);
        run(weak, new RecoveryTest(weak, weak, SP, 1), 0.15, new double[] { 6 }, 60);
        double weakTime = num("Last_RecoveryTime");

        var strong = roller(0.00128, 0.0017636);
        run(strong, new RecoveryTest(strong, strong, SP, 1), 0.15, new double[] { 6 }, 60);
        double strongTime = num("Last_RecoveryTime");

        System.out.printf("  weak %.2f s vs strong %.2f s%n", weakTime, strongTime);
        assertTrue(strongTime < weakTime, "strong " + strongTime + " should beat weak " + weakTime);
    }

    @Test
    void failsWhenNoLoadIsApplied() {
        var r = roller(0.00128, 0.0017636);
        run(r, new RecoveryTest(r, r, SP, 1), 0, new double[] {}, 70);
        assertTrue(key("Status").startsWith("FAILED"), key("Status"));
    }
}
