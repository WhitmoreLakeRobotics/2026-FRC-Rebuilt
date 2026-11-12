package frc.robot.utils.motor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * A SparkMax running a position loop on its primary encoder.
 *
 * Moves toward maxPosition use the forward slot (slot 0, settings.gains()); moves toward
 * minPosition use the reverse slot (slot 1, settings.reverseGains()). setPosition() picks the
 * slot from where the mechanism is now, so a subsystem never deals with slots.
 *
 * Targets are clamped to [minPosition, maxPosition], and soft limits are set there too when
 * the settings enable them, so a tuner stepping the mechanism around cannot drive it past travel.
 *
 * applyGains(Gains) writes the forward slot, which is what a direction-agnostic tuner expects;
 * applyGains(forward, reverse) sets both.
 */
public class SparkPositionMotor implements PositionLoop {

    private static final ClosedLoopSlot FORWARD = ClosedLoopSlot.kSlot0;
    private static final ClosedLoopSlot REVERSE = ClosedLoopSlot.kSlot1;

    private final SparkMax motor;
    private final PositionMotorSettings settings;
    private double target = 0;

    public SparkPositionMotor(PositionMotorSettings settings) {
        this.settings = settings;
        this.motor = new SparkMax(settings.canId(), MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(settings.inverted());
        config.idleMode(settings.brake() ? IdleMode.kBrake : IdleMode.kCoast);
        config.smartCurrentLimit(settings.stallAmps(), settings.freeAmps());
        if (settings.voltageComp() > 0) {
            config.voltageCompensation(settings.voltageComp());
        }
        if (settings.rampRate() > 0) {
            config.closedLoopRampRate(settings.rampRate());
        }
        config.encoder.positionConversionFactor(settings.positionFactor());
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.outputRange(-settings.maxOutput(), settings.maxOutput(), FORWARD);
        config.closedLoop.outputRange(-settings.maxOutput(), settings.maxOutput(), REVERSE);
        writeGains(config, settings.gains(), FORWARD);
        writeGains(config, settings.reverseGains(), REVERSE);

        boolean finiteRange = Double.isFinite(settings.minPosition()) && Double.isFinite(settings.maxPosition());
        config.softLimit.forwardSoftLimitEnabled(settings.softLimits() && finiteRange);
        config.softLimit.reverseSoftLimitEnabled(settings.softLimits() && finiteRange);
        if (finiteRange) {
            config.softLimit.forwardSoftLimit(settings.maxPosition());
            config.softLimit.reverseSoftLimit(settings.minPosition());
        }
        settings.extra().accept(config);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private static void writeGains(SparkMaxConfig config, Gains g, ClosedLoopSlot slot) {
        config.closedLoop.pid(g.kP(), g.kI(), g.kD(), slot);
        config.closedLoop.feedForward.kV(g.kV(), slot);
    }

    // ---- PositionLoop -------------------------------------------------------

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public void setPosition(double position) {
        target = Math.max(settings.minPosition(), Math.min(settings.maxPosition(), position));
        ClosedLoopSlot slot = target >= getPosition() ? FORWARD : REVERSE;
        motor.getClosedLoopController().setSetpoint(target, ControlType.kPosition, slot);
    }

    @Override
    public double getTargetPosition() {
        return target;
    }

    @Override
    public double minPosition() {
        return settings.minPosition();
    }

    @Override
    public double maxPosition() {
        return settings.maxPosition();
    }

    @Override
    public void resetPosition(double position) {
        motor.getEncoder().setPosition(position);
    }

    // ---- ClosedLoop ---------------------------------------------------------

    @Override
    public String name() {
        return settings.name();
    }

    /** Forward-slot gains. */
    @Override
    public Gains getGains() {
        return readGains(FORWARD);
    }

    public Gains getReverseGains() {
        return readGains(REVERSE);
    }

    private Gains readGains(ClosedLoopSlot slot) {
        var cl = motor.configAccessor.closedLoop;
        return new Gains(cl.getP(slot), cl.getI(slot), cl.getD(slot), cl.feedForward.getkV(slot));
    }

    /** Writes the forward slot only. */
    @Override
    public void applyGains(Gains forward) {
        SparkMaxConfig config = new SparkMaxConfig();
        writeGains(config, forward, FORWARD);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void applyGains(Gains forward, Gains reverse) {
        SparkMaxConfig config = new SparkMaxConfig();
        writeGains(config, forward, FORWARD);
        writeGains(config, reverse, REVERSE);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void resetIntegral() {
        motor.getClosedLoopController().setIAccum(0);
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    // ---- Diagnostics --------------------------------------------------------

    public double getVelocity()      { return motor.getEncoder().getVelocity(); }
    public double getOutputCurrent() { return motor.getOutputCurrent(); }
    public double getAppliedOutput() { return motor.getAppliedOutput(); }

    public PositionMotorSettings settings() {
        return settings;
    }
}
