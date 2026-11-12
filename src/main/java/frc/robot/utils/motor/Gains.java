package frc.robot.utils.motor;

/**
 * Closed-loop gains for one controller slot.
 *
 * This is the part of a motor's settings that tuning changes, so it is its own record:
 * the auto-tuners read and write a Gains, and the subsystem's settings hold the starting one.
 * kV is the velocity feedforward (output per unit of setpoint); leave it 0 for a position loop.
 */
public record Gains(double kP, double kI, double kD, double kV) {

    public static final Gains NONE = new Gains(0, 0, 0, 0);

    public static Gains pid(double kP, double kI, double kD) {
        return new Gains(kP, kI, kD, 0);
    }

    public Gains withP(double p) { return new Gains(p, kI, kD, kV); }
    public Gains withI(double i) { return new Gains(kP, i, kD, kV); }
    public Gains withD(double d) { return new Gains(kP, kI, d, kV); }
    public Gains withV(double v) { return new Gains(kP, kI, kD, v); }

    @Override
    public String toString() {
        return String.format("kP=%.6f kI=%.7f kD=%.7f kV=%.6f", kP, kI, kD, kV);
    }
}
