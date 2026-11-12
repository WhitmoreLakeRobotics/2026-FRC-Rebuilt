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
 * A SparkMax running a velocity loop on its primary encoder, slot 0.
 *
 * This is the only class that knows the REVLib calls for a velocity loop. Subsystems hold one
 * of these per velocity-controlled motor and talk to it through VelocityLoop; tuning commands
 * get it through the subsystem's accessor, e.g.
 *   new AutoTuneVelocity(m_intake.roller(), m_intake, 4800)
 *
 * Gains applied with applyGains() are not persisted. The gains in the settings are written
 * once, with persist, in the constructor.
 */
public class SparkVelocityMotor implements VelocityLoop {

    private static final ClosedLoopSlot SLOT = ClosedLoopSlot.kSlot0;

    private final SparkMax motor;
    private final VelocityMotorSettings settings;
    private double target = 0;

    public SparkVelocityMotor(VelocityMotorSettings settings) {
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
        config.encoder.velocityConversionFactor(settings.velocityFactor());
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(settings.gains().kP(), settings.gains().kI(), settings.gains().kD(), SLOT);
        config.closedLoop.feedForward.kV(settings.gains().kV(), SLOT);
        settings.extra().accept(config);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ---- VelocityLoop -------------------------------------------------------

    @Override
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public void setVelocity(double velocity) {
        target = velocity;
        if (velocity == 0) {
            motor.set(0);
        } else {
            motor.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity, SLOT);
        }
    }

    @Override
    public double getTargetVelocity() {
        return target;
    }

    // ---- ClosedLoop ---------------------------------------------------------

    @Override
    public String name() {
        return settings.name();
    }

    @Override
    public Gains getGains() {
        var cl = motor.configAccessor.closedLoop;
        return new Gains(cl.getP(SLOT), cl.getI(SLOT), cl.getD(SLOT), cl.feedForward.getkV(SLOT));
    }

    @Override
    public void applyGains(Gains g) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(g.kP(), g.kI(), g.kD(), SLOT);
        config.closedLoop.feedForward.kV(g.kV(), SLOT);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void resetIntegral() {
        motor.getClosedLoopController().setIAccum(0);
    }

    @Override
    public void stop() {
        setVelocity(0);
    }

    // ---- Diagnostics --------------------------------------------------------

    public double getOutputCurrent() { return motor.getOutputCurrent(); }
    public double getAppliedOutput() { return motor.getAppliedOutput(); }
    public double getTemperature()   { return motor.getMotorTemperature(); }

    /** The settings this motor was built with, e.g. to show the starting gains next to tuned ones. */
    public VelocityMotorSettings settings() {
        return settings;
    }
}
