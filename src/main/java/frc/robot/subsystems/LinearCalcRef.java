package frc.robot.subsystems;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;

public class LinearCalcRef {

    private final double rpm;
    private final double distance;
    // Removed <?> because Unit is not a generic type in this version
    private final Unit unitOfMeasure;

    public LinearCalcRef(double rpm, double distance, DistanceUnit units) {
        this.rpm = rpm;
        this.distance = distance;
        this.unitOfMeasure = units;
    }

    public LinearCalcRef(double rpm, double distance, AngleUnit units) {
        this.rpm = rpm;
        this.distance = distance;
        this.unitOfMeasure = units;
    }

    public double getRPM() {
        return rpm;
    }

    public double getDistance() {
        // Ensure this matches the field name 'distance' exactly
        return distance;
    }

    /**
     * Casts the unit to the specific type expected by the caller.
     */
    @SuppressWarnings("unchecked")
    public <U extends Unit> U getUnitOfMeasure() {
        return (U) unitOfMeasure;
    }
}
