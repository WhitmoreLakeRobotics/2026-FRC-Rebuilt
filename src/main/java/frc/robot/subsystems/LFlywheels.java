package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MultiMotorPIDTunable;

/**
 * LFlywheels
 * 
 * Controls a pair of flywheel motors (left and right) for a launch tower.
 * Implements MultiMotorPIDTunable so AutoTuneMultiMotor can tune each
 * motor's PID gains independently.
 * 
 * Motor index convention used by the tuner:
 *   0 = Left motor  (TowerLM)
 *   1 = Right motor (TowerRM)
 * 
 * PID gain arrays passed in constructor (index order: kP, kI, kD, kF):
 *   flyWheelLeft  -> applied to TowerLM slot 0
 *   flyWheelRight -> applied to TowerRM slot 0
 */
public class LFlywheels extends SubsystemBase implements MultiMotorPIDTunable {

    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    private SparkMax TowerRM;
    private SparkMax TowerLM;
    private SparkMaxConfig TowerConfig = new SparkMaxConfig();

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    private double[] flyWheelRight;
    private double[] flyWheelLeft;

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------

    private double TargetRPM = 0.0;
    private FlywheelStatus status = FlywheelStatus.STOPPED;
    private final double RPM_Tolerance = 20.0;

    public static final double MaxRpm = 0;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * @param TowerMotorR_canid CAN ID for the right flywheel motor
     * @param TowerMotorL_canid CAN ID for the left flywheel motor
     * @param flyWheelRight     PID gains for right motor: {kP, kI, kD, kF}
     * @param flyWheelLeft      PID gains for left motor:  {kP, kI, kD, kF}
     */
    public LFlywheels(int TowerMotorR_canid, int TowerMotorL_canid,
                      double[] flyWheelRight, double[] flyWheelLeft) {
        TowerRM = new SparkMax(TowerMotorR_canid, MotorType.kBrushless);
        TowerLM = new SparkMax(TowerMotorL_canid, MotorType.kBrushless);
        this.flyWheelRight = flyWheelRight;
        this.flyWheelLeft  = flyWheelLeft;
        configrRTowerMotor();
        configLTowerMotor();
    }

    // -----------------------------------------------------------------------
    // Periodic
    // -----------------------------------------------------------------------

    @Override
    public void periodic() {
        switch (status) {
            case RAMPING:
                boolean rightAtSpeed = Math.abs(TargetRPM - TowerRM.getEncoder().getVelocity()) < RPM_Tolerance;
                boolean leftAtSpeed  = Math.abs(TargetRPM - TowerLM.getEncoder().getVelocity()) < RPM_Tolerance;
                if (rightAtSpeed && leftAtSpeed) {
                    status = (TargetRPM > 0) ? FlywheelStatus.AT_SPEED : FlywheelStatus.REVERSE;
                }
                break;
            case REVERSE:
            case AT_SPEED:
            case STOPPED:
            default:
                break;
        }
    }

    @Override
    public void simulationPeriodic() {}

    // -----------------------------------------------------------------------
    // Normal flywheel control (used by SetTarget command)
    // -----------------------------------------------------------------------

    public void setRPM(double rpm) {
        if (rpm != 0) {
            TargetRPM = rpm;
            TowerRM.getClosedLoopController().setReference(rpm, ControlType.kVelocity);
            TowerLM.getClosedLoopController().setReference(rpm, ControlType.kVelocity);
            status = FlywheelStatus.RAMPING;
        } else {
            status = FlywheelStatus.STOPPED;
            TowerRM.set(0);
            TowerLM.set(0);
        }
    }

    // -----------------------------------------------------------------------
    // Getters
    // -----------------------------------------------------------------------

    public double getRPM()           { return TargetRPM; }
    public FlywheelStatus getStatus(){ return status; }
    public double getRMotorRPM()     { return TowerRM.getEncoder().getVelocity(); }
    public double getLMotorRPM()     { return TowerLM.getEncoder().getVelocity(); }

    // -----------------------------------------------------------------------
    // MultiMotorPIDTunable implementation
    // -----------------------------------------------------------------------

    /**
     * Two motors: index 0 = Left, index 1 = Right.
     */
    @Override
    public int getMotorCount() { return 2; }

    @Override
    public String getMotorName(int motorIndex) {
        return motorIndex == 0 ? "Left" : "Right";
    }

    /**
     * Per-motor RPM reading used by the auto-tuner.
     */
    @Override
    public double getMeasurement(int motorIndex) {
        return motorIndex == 0
            ? TowerLM.getEncoder().getVelocity()
            : TowerRM.getEncoder().getVelocity();
    }

    /**
     * Per-motor velocity reference used by the auto-tuner.
     * Bypasses setRPM() so the tuner controls one motor at a time.
     */
    @Override
    public void setOutput(int motorIndex, double output) {
        if (motorIndex == 0) {
            TowerLM.getClosedLoopController()
                .setReference(output, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        } else {
            TowerRM.getClosedLoopController()
                .setReference(output, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }
    }

    /**
     * Writes recommended gains to one motor after auto-tuning completes.
     * 
     * Uses kNoPersistParameters intentionally — gains are NOT saved to
     * flash until you manually call persist. This lets you test multiple
     * tuning runs without wearing out the SparkMAX flash memory.
     * 
     * Once happy with the values, update the flyWheelLeft / flyWheelRight
     * arrays in your Constants file and redeploy with kPersistParameters.
     */
    @Override
    public void applyGains(int motorIndex, double kP, double kI, double kD, double kF) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
        config.closedLoop.feedForward.kV(kF);

        SparkMax motor = (motorIndex == 0) ? TowerLM : TowerRM;
        motor.configure(config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

    // -----------------------------------------------------------------------
    // PIDTunable single-motor fallback (required by interface)
    // -----------------------------------------------------------------------

    /** Average RPM of both motors — used by manual TunePID command. */
    @Override
    public double getMeasurement() {
        return (getLMotorRPM() + getRMotorRPM()) / 2.0;
    }

    /** Sets both motors to the same RPM — used by manual TunePID command. */
    @Override
    public void setOutput(double output) {
        setRPM(output);
    }

    @Override
    public String getTunableName() { return "LFlywheels"; }

    // -----------------------------------------------------------------------
    // Motor configuration (unchanged from original)
    // -----------------------------------------------------------------------

    private void configLTowerMotor() {
        TowerConfig.encoder.positionConversionFactor(1);
        TowerConfig.inverted(false);
        TowerConfig.softLimit.forwardSoftLimitEnabled(false);
        TowerConfig.softLimit.reverseSoftLimitEnabled(false);
        TowerConfig.idleMode(IdleMode.kCoast);
        TowerConfig.encoder.quadratureAverageDepth(4);
        TowerConfig.closedLoop.pid(flyWheelLeft[0], flyWheelLeft[1], flyWheelLeft[2], ClosedLoopSlot.kSlot0);
        TowerConfig.closedLoop.feedForward.kV(flyWheelLeft[3]);
        TowerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        TowerConfig.smartCurrentLimit(35, 45);
        TowerLM.configure(TowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configrRTowerMotor() {
        TowerConfig.encoder.positionConversionFactor(1);
        TowerConfig.inverted(true);
        TowerConfig.softLimit.forwardSoftLimitEnabled(false);
        TowerConfig.softLimit.reverseSoftLimitEnabled(false);
        TowerConfig.idleMode(IdleMode.kCoast);
        TowerConfig.encoder.quadratureAverageDepth(4);
        TowerConfig.closedLoop.pid(flyWheelRight[0], flyWheelRight[1], flyWheelRight[2], ClosedLoopSlot.kSlot0);
        TowerConfig.closedLoop.feedForward.kV(flyWheelRight[3]);
        TowerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        TowerConfig.smartCurrentLimit(35, 45);
        TowerRM.configure(TowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // -----------------------------------------------------------------------
    // Status enum
    // -----------------------------------------------------------------------

    public enum FlywheelStatus {
        RAMPING,
        REVERSE,
        AT_SPEED,
        STOPPED
    }
}
