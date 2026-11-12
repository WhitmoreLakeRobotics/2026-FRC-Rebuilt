package frc.robot.utils.motor;

import java.util.function.Consumer;

import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Everything a SparkPositionMotor needs to know about one mechanism, in one place.
 *
 * A position loop often wants different gains in each direction (gravity, a spring):
 * `gains` drives moves toward maxPosition and `reverseGains` moves toward minPosition.
 * Leave reverseGains unset to use the same gains both ways.
 *
 * Example:
 *   static final PositionMotorSettings EXTEND =
 *       PositionMotorSettings.builder(CanIds.MOTORS.INTAKE_EXTEND.canId, "Intake/Extend")
 *           .inverted(true).brake().currentLimit(40).voltageComp(9)
 *           .range(-4, 15.1).maxOutput(0.5).rampRate(0.15)
 *           .gains(Gains.pid(0.055, 0, 0)).reverseGains(Gains.pid(0.07, 0, 0))
 *           .build();
 *
 * @param positionFactor encoder position is multiplied by this; 1.0 = motor rotations
 * @param softLimits     when true, controller soft limits are enabled at min/max
 * @param maxOutput      closed-loop output is clamped to +/- this (0..1)
 */
public record PositionMotorSettings(
        int canId,
        String name,
        boolean inverted,
        boolean brake,
        int stallAmps,
        int freeAmps,
        double voltageComp,
        double positionFactor,
        double minPosition,
        double maxPosition,
        boolean softLimits,
        double maxOutput,
        double rampRate,
        Gains gains,
        Gains reverseGains,
        Consumer<SparkMaxConfig> extra) {

    public static Builder builder(int canId, String name) {
        return new Builder(canId, name);
    }

    public static final class Builder {
        private final int canId;
        private final String name;
        private boolean inverted = false;
        private boolean brake = true;
        private int stallAmps = 40;
        private int freeAmps = 40;
        private double voltageComp = 0;
        private double positionFactor = 1.0;
        private double minPosition = Double.NEGATIVE_INFINITY;
        private double maxPosition = Double.POSITIVE_INFINITY;
        private boolean softLimits = false;
        private double maxOutput = 1.0;
        private double rampRate = 0;
        private Gains gains = Gains.NONE;
        private Gains reverseGains = null;
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
        public Builder positionFactor(double f)          { positionFactor = f; return this; }
        /** Travel limits in position units; also where soft limits go if enabled. */
        public Builder range(double min, double max)     { minPosition = min; maxPosition = max; return this; }
        public Builder softLimits(boolean enabled)       { softLimits = enabled; return this; }
        public Builder maxOutput(double fraction)        { maxOutput = fraction; return this; }
        public Builder rampRate(double seconds)          { rampRate = seconds; return this; }
        public Builder gains(Gains g)                    { gains = g; return this; }
        public Builder reverseGains(Gains g)             { reverseGains = g; return this; }
        public Builder extra(Consumer<SparkMaxConfig> c) { extra = c; return this; }

        public PositionMotorSettings build() {
            return new PositionMotorSettings(canId, name, inverted, brake, stallAmps, freeAmps,
                    voltageComp, positionFactor, minPosition, maxPosition, softLimits, maxOutput,
                    rampRate, gains, reverseGains == null ? gains : reverseGains, extra);
        }
    }
}
