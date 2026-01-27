package frc.robot.subsystems;
import edu.wpi.first.units.DistanceUnit;


public class LinearCalcRef {

    double RPM;
    double Distance;
    DistanceUnit unitOfMeasure;

    public LinearCalcRef (double rpm, double distance,  DistanceUnit units) {
        this.RPM = rpm;
        this.Distance = distance;
        this.unitOfMeasure = units;

    }

    public LinearCalcRef (int rpm, int distance,  DistanceUnit units) {
        this.RPM = rpm;
        this.Distance = distance;
        this.unitOfMeasure = units;
    }


    public double getRPM () {
        return RPM;
    }

    public  double getDistance() {
        return Distance;
    }

    public DistanceUnit getUnitofMeasure() {
        return unitOfMeasure;
    }

    

}
