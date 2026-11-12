package frc.robot.utils.motor;

import java.util.function.Consumer;

import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Everything a SparkVelocityMotor needs to know about one mechanism, in one place.
 *
 * Only the knobs that change between mechanisms are here. Anything else REVLib offers
 * can be set through `extra`, which runs on the config before it is written.
 *
 * Example:
 *   static final VelocityMotorSettings ROLLER =
 *       VelocityMotorSettings.builder(CanIds.MOTORS.INTAKE_MOTOR.canId, "Intake/Roller")
 *           .inverted(true).coast().currentLimit(40, 70).voltageComp(10)
 *           .gains(new Gains(0.016384, 0, 0, 0.002097))
 *           .build();
 *
 * @param velocityFactor encoder velocity is multiplied by this; 1.0 = motor RPM
 * @param rampRate       seconds from 0 to full output in closed loop; 0 disables
 */
public record VelocityMotorSettings(
        int canId,
        String name,
        boolean inverted,
        boolean brake,
        int stallAmps,
        int freeAmps,
        double voltageComp,
        double velocityFactor,
        double rampRate,
        Gains gains,
        Consumer<SparkMaxConfig> extra) {

    public static Builder builder(int canId, String name) {
        return new Builder(canId, name);
    }

    public static final class Builder {
        private final int canId;
        private final String name;
        private boolean inverted = false;
        private boolean brake = false;
        private int stallAmps = 40;
        private int freeAmps = 40;
        private double voltageComp = 0;
        private double velocityFactor = 1.0;
        private double rampRate = 0;
        private Gains gains = Gains.NONE;
        private Consumer<SparkMaxConfig> extra = c -> {};

        private Builder(int canId, String name) {
            this.canId = canId;
            this.name = name;
        }

        public Builder inverted(boolean v)               { inverted = v; return this; }
        public Builder brake()                           { brake = true; return this; }
        public Builder coast()                           { brake = false; return this; }
        public Builder currentLimit(int stall, int free) { stallAmps = stall; freeAmps = free; return this; }
        public Builder currentLimit(int amps)            { return currentLimit(amps, amps); }
        public Builder voltageComp(double volts)         { voltageComp = volts; return this; }
        public Builder velocityFactor(double f)          { velocityFactor = f; return this; }
        public Builder rampRate(double seconds)          { rampRate = seconds; return this; }
        public Builder gains(Gains g)                    { gains = g; return this; }
        public Builder extra(Consumer<SparkMaxConfig> c) { extra = c; return this; }

        public VelocityMotorSettings build() {
            return new VelocityMotorSettings(canId, name, inverted, brake, stallAmps, freeAmps,
                    voltageComp, velocityFactor, rampRate, gains, extra);
        }
    }
}
