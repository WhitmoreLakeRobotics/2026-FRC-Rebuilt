package frc.robot.utils;


import com.revrobotics.spark.SparkClosedLoopController;

/**
 * PIDTunable
 * 
 * A contract that any subsystem can implement to opt into
 * the PID tuning system. The TunePID and AutoTuneMultiMotor
 * commands use this interface so they work generically across
 * any subsystem without needing to know the hardware details.
 * 
 * Place this file at:
 *   src/main/java/frc/robot/util/PIDTunable.java
 * 
 * To use: add "implements PIDTunable" to your subsystem class
 * and add the three @Override methods.
 */
public interface PIDTunable {

    /**
     * Returns the current measurement for this subsystem.
     * Examples:
     *   Intake    -> getCurrPos()
     *   Flywheels -> getLMotorRPM()
     *   Hopper    -> getBeltMotorVelocity()
     */
    //SparkClosedLoopController getLoopController();

    double getCurrVelocity();
    double getTargVelocity();
    double getP();
    double getI();
    double getD();
    double getF();
    double getIACC();
    
    void setCurrVelocity();
    void setTargVelocity();
    void setP();
    void setI();
    void setD();
    void setF();
    void setIACC();

    /**
     * Returns a short name used to namespace SmartDashboard keys.
     * Example: "Intake" produces keys like "Intake/kP", "Intake/Error"
     */
    String getTunableName();

}

