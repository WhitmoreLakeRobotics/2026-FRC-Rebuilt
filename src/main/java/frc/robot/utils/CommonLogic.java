package frc.utils;

//import java.util.Optional;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import frc.robot.RobotContainer;

public class CommonLogic {

  public static double CapValue(double currentValue, double lowValue, double hiValue) {
    // logic to cap the motor power between a good range
    double retValue = currentValue;

    if (currentValue < lowValue) {
      retValue = lowValue;
    }

    if (currentValue > hiValue) {
      retValue = hiValue;
    }

    return retValue;
  }

  public static final double joyDeadBand(double joy, double deadband) {

    double retValue = joy;
    if (Math.abs(retValue) < Math.abs(deadband)) {
      retValue = 0;
    }
    return Math.pow(retValue, 2) * Math.signum(joy);
  }

  public static final boolean isInRange(double currentValue, double targetValue, double Tol) {

    double loVal = targetValue - Tol;
    double hiVal = targetValue + Tol;
    boolean retValue = false;

    if (currentValue > loVal && currentValue < hiVal) {
      retValue = true;
    }
    return retValue;
  }

  public static double getTime() {
    return (System.nanoTime() / Math.pow(10, 9)); // returns time in seconds
  }

}
