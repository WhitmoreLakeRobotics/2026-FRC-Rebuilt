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
import frc.robot.subsystems.LTurret.TurretStatus;

public class LFlywheels extends SubsystemBase {

    public static final double MaxRpm = 0;

    private SparkMax TowerRM;
    private SparkMax TowerLM;
    private double TargetRPM = 0.0;
    private FlywheelStatus status = FlywheelStatus.STOPPED;
    private final double RPM_Tolerance = 20.0;
    private SparkMaxConfig TowerConfig = new SparkMaxConfig();

    //private double[] rightPIDF;
    //private double[] leftPIDF;

    ClosedLoopSlot flywheelPidSlot = ClosedLoopSlot.kSlot0;

    public LFlywheels(int TowerMotorR_canid, int TowerMotorL_canid, double[] flyWheelRight, double[] flyWheelLeft) {

        TowerRM = new SparkMax(TowerMotorR_canid, MotorType.kBrushless);
        TowerLM = new SparkMax(TowerMotorL_canid, MotorType.kBrushless);
        //this.rightPIDF = flyWheelRight;
        //this.leftPIDF = flyWheelLeft;

        TowerConfig = getSparkMaxConfig();

        // setup the Right Motor
        TowerConfig.inverted(true);
        TowerConfig.closedLoop.pid(flyWheelRight[0], flyWheelRight[1],flyWheelRight[2], flywheelPidSlot);
        TowerConfig.closedLoop.feedForward.kV((flyWheelRight[3]));
        TowerRM.configure(TowerConfig,ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // setup the Left Motor
        TowerConfig.inverted(false);
        TowerConfig.closedLoop.pid(flyWheelLeft[0], flyWheelLeft[1],flyWheelLeft[2], flywheelPidSlot);
        TowerConfig.closedLoop.feedForward.kV((flyWheelLeft[3]));
        TowerLM.configure(TowerConfig,ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        switch (status) {
            case RAMPING:
                if ((Math.abs(TargetRPM -
                    TowerRM.getEncoder().getVelocity()) < RPM_Tolerance)
                 && (Math.abs(TargetRPM -
                    TowerLM.getEncoder().getVelocity()) < RPM_Tolerance)) {
                    if(TargetRPM > 0){
                        status = FlywheelStatus.AT_SPEED;
                    } else{
                        status = FlywheelStatus.REVERSE;
                    }
                }
                break;
            case REVERSE:

                break;
            case AT_SPEED:

                break;

            case STOPPED:

                break;
            default:
                break;
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }
        public double getRPM() {
            return TargetRPM;
        }
    public FlywheelStatus getStatus() {
            return status;
        }


        public double getRMotorRPM() {
            return TowerRM.getEncoder().getVelocity();
        }
        public double getLMotorRPM() {
            return TowerLM.getEncoder().getVelocity();
        }

        public void setRPM(double rpm) {
            if (rpm != 0) {
            TargetRPM = rpm;
            TowerRM.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
            TowerLM.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
            status = FlywheelStatus.RAMPING;

            }else{
                status = FlywheelStatus.STOPPED;
                TowerRM.set(0);
                TowerLM.set(0);
            }



        }


    private SparkMaxConfig getSparkMaxConfig () {
        SparkMaxConfig returnConfig = new SparkMaxConfig();

        returnConfig.encoder.positionConversionFactor(1);
        returnConfig.softLimit.forwardSoftLimitEnabled(false);
        //returnConfig.softLimit.reverseSoftLimit(0);
        returnConfig.softLimit.reverseSoftLimitEnabled(false);
        returnConfig.idleMode(IdleMode.kCoast);
        // returnConfig.openLoopRampRate(0.15);
        // returnConfig.limitSwitch.forwardLimitSwitchEnabled(false);
        // returnConfig.limitSwitch.reverseLimitSwitchEnabled(false);
        // returnConfig.closedLoop.maxOutput(1.0);
        // returnConfig.closedLoop.minOutput(-1.0);
        // returnConfig.closedLoopRampRate(0.075);
        // returnConfig.voltageCompensation(9.0);
        // returnConfigclosedLoop.maxMotion.cruiseVelocity(0);
        // returnConfig.closedLoop.maxMotion.maxAcceleration(0);
        // returnConfig.closedLoop.maxMotion.maxAcceleration(5000, TowerConfig_CLOSED_LOOP_SLOT_DOWN);
        // returnConfig.closedLoop.maxMotion.maxVelocity(5000, TowerConfig_CLOSED_LOOP_SLOT_DOWN);
        // returnConfig.closedLoop.maxMotion.allowedClosedLoopError(TowerConfigPosTol, TowerConfig_CLOSED_LOOP_SLOT_DOWN);
        // returnConfig.closedLoop.pidf(0.4, 0.0, 0.0, 0.0, TowerConfig_CLOSED_LOOP_SLOT_DOWN);
        // returnConfig.closedLoop.maxMotion.maxAcceleration(5000, Tower_CLOSED_LOOP_SLOT_UP);
        // returnConfig.closedLoop.maxMotion.maxVelocity(2000, Tower_CLOSED_LOOP_SLOT_UP);
        // returnConfig.closedLoop.maxMotion.allowedClosedLoopError(TowerConfigPosTol, Tower_CLOSED_LOOP_SLOT_UP);
        // returnConfig.closedLoop.pidf(0.4, 0.0, 0.0, 0.0, Tower_CLOSED_LOOP_SLOT_UP);

        returnConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // returnConfig.smartCurrentLimit(50);
        returnConfig.smartCurrentLimit(35, 50);

        return returnConfig;

    }

    //need to create enum for status of flywheels
    public enum FlywheelStatus {
        RAMPING,
        REVERSE,
        AT_SPEED,
        STOPPED;

    }


}

