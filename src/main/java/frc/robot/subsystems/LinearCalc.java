package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
//import static edu.wpi.first.units.Units.Centermeter;

import java.util.ArrayList;
import java.util.Comparator;

public class LinearCalc {

    // Create the ArrayList for the storage of data.
    ArrayList<LinearCalcRef> references = new ArrayList<>();
    ArrayList<LinearCalcSection> sections = new ArrayList<>();
    private double minimumRPM = 0.0;
    private boolean minRPMSet = false;
    private double maximumRPM = 1000000;
    private boolean maxRPMSet = false;

    LinearCalc() {

    }

    LinearCalc(LinearCalcRef shortShot, LinearCalcRef longShot) {
        references.add(shortShot);
        references.add(longShot);
        Calculate();

    }

    // Adds a new reference to the list.
    public void add(LinearCalcRef newRef) {

        for (int i = 0; i < references.size(); i++) {
            if (newRef.getDistance() == references.get(i).getDistance()) {
                throw new IllegalArgumentException("LinearCalc duplicate distance detected: RPM->" + newRef.getRPM()
                        + " Distance-> " + newRef.getDistance());
            }
        }

        references.add(newRef);
    }

    public void setMinimumRPM(double minRPM) {
        minimumRPM = minRPM;
        minRPMSet = true;
    }

    public void unsetMinimumRPM() {
        minRPMSet = false;
    }

    public void setMaximumRPM(double maxRPM) {
        maximumRPM = maxRPM;
        maxRPMSet = true;
    }

    public void unsetMaximumRPM() {
        maxRPMSet = false;
    }

    public int getRefCount() {
        return references.size();
    }

    public int getSectionCount() {
        return sections.size();
    }

    public void Calculate() {
        // sort the references by distance
        sections.clear();
        references.sort(Comparator.comparing(LinearCalcRef::getDistance));

        // iterate over the references. NOTE there must be more than 2 references

        if (references.size() < 2) {
            throw new ArrayIndexOutOfBoundsException(
                    "Not enough data:  Must provide atleast 2 LinearCalReferences. \n" +
                            "We have " + references.size() + ".");
        }

		for (int i = 0; i < (references.size() - 1); i++) {
            sections.add(new LinearCalcSection(references.get(i), references.get(i + 1)));
        }

    }

    public double getRPM(double distance) {

        double rpm = 0.0;

        // If distance is less than short shot of first section then use first section
        if (distance < sections.get(0).getShortDistance()) {
            rpm = sections.get(0).getRPM(distance);
            //System.out.println("Under Distance " + distance);
        }

        if (distance > sections.get(sections.size() - 1).getLongDistance()) {
            rpm = sections.get(sections.size() - 1).getRPM(distance);
            //System.out.println("Over Distance " + distance);
        }

        for (int i = 0; i < sections.size(); i++) {
            if ((sections.get(i).getShortDistance() <= distance) &&
                    (sections.get(i).getLongDistance() >= distance)) {
                rpm = sections.get(i).getRPM(distance);
            }

        }

        // If the value has been set then clamp the minimum value
        if (minRPMSet) {
            if (rpm < minimumRPM) {
                rpm = minimumRPM;
            }
        }

        // if the value has been set then clamp the maximum value
        if (maxRPMSet) {
            if (rpm > maximumRPM) {
                rpm = maximumRPM;
            }
        }

        return rpm;

    }

    public double getLongDistance() {
        return sections.get(sections.size() - 1).getLongDistance();
    }

    public void printReferences() {

        System.out.printf("Number of references  %d\n", references.size());

        for (int i = 0; i < references.size(); i++) {
            System.out.printf("Distance %6.2f     RPM %6.2f%n", references.get(i).getDistance(),
                    references.get(i).getRPM());
        }

    }

    public boolean isReferenceDist(double refDist){

        boolean retValue = false;
        for (int i = 0; i < references.size(); i++) {
            if (refDist == references.get(i).getDistance()) {
                retValue = true;
                break;
            }
        }
        return retValue;

    }

    public boolean isReferenceRPM(double refRPM){

        boolean retValue = false;
        for (int i = 0; i < references.size(); i++) {
            if (refRPM == references.get(i).getDistance()) {
                retValue = true;
                break;
            }
        }
        return retValue;

    }


    public void printSections() {

        System.out.printf("Number of sections  %d\n", sections.size());
        for (int i = 0; i < sections.size(); i++) {
            System.out.printf("Short Distance %6.2f    RPM %6.2f\t", sections.get(i).getShortDistance(),
                    sections.get(i).getShortRPM());
            System.out.printf("Long Distance %6.2f     RPM %6.2f%n", sections.get(i).getLongDistance(),
                    sections.get(i).getLongRPM());
        }
    }


    public double getMikeRPM(double distanceInches, double turretAngleDegrees) {

        double baselineDistanceInches = 131.0;
        double baselineRPM = 3337.0;

        // Distance scaling: +1% per inch farther than baseline
        double deltaInches = distanceInches - baselineDistanceInches;
        double distanceScale = 1.0 + (0.01 * deltaInches);

        // Angle correction: 0–135° → 0%, 135–180° → +15%
        double a = Math.abs(turretAngleDegrees);
        double angleScale = ((a > 135.0) && (a < 225.0)) ? 1.15 : 1.0;

        // Final RPM
        return baselineRPM * distanceScale * angleScale;
    }

    public double getPollyRPM (double distanceInches) {
        // These coefficients specifically drop the left side (x < 125)
    // while maintaining the curve's height around 160-170.
    double a = -0.5284;  // Increased 'bend'
    double b = 175.2;    // Shifted peak to the right
    double c = -10450.0; // Adjusted vertical offset
        // Formula: y = ax^2 + bx + c
        return (a * Math.pow(distanceInches, 2)) + (b * distanceInches) + c;
    }


    public double getVelRPM (double rangeInches) {

        double ANGLE_RAD = Math.toRadians(51);
        double cosTheta = Math.cos(ANGLE_RAD);
        double tanTheta = Math.tan(ANGLE_RAD);
        double G = 386.1; //  (inches/sec^2)
        double TARGET_HEIGHT_DELTA =  72 + 6 - 21;   // Hub Height + clearance - Robot Height

        // Physics formula for velocity (v)
        double numerator = G * Math.pow(rangeInches, 2);
        double denominator = 2 * Math.pow(cosTheta, 2) * ((rangeInches * tanTheta )- TARGET_HEIGHT_DELTA);

        // Check for "Impossible Shot" (square root of a negative)
        if (denominator <= 0) return 0.0;

        double velocity = Math.sqrt(numerator / denominator);
        double slipFactor = 2.145;
        double VELOCITY_TO_RPM = ((Math.PI * 2)/60)  ;   // (Math.PI * 2)/60 = the linear inches/second

        // Convert linear velocity to RPM
        return velocity * VELOCITY_TO_RPM * 60 * slipFactor;   //Vel is in inches/sec   and RPM is rotations/Min
    }
/*
     * Testing @ fowlerville
     * 10 feet
     *
     * 10.9 3337 0
     * 10.7 3370 45L
     *
     * 10.9 3415 90L
     * 10.9 3440 90L
     * 17.0 4800 corner shot
     *
     * 129 in 3600 270R
     *
     *
     * 138 3650 0 145tape
     *
     * 132 3520 0 135 tape 630
     * 132.9 4150 180 135 tape
     * 134.3 4200 135 138tape 680
     *
     * 144 3815 0 156tape
     * 144 4850 180 156tape 1035
     * 147.74 4850 135 160tape 1035
     *
     *
     * 156 4056 0 167tape
     * 166 5051 180 165tape 995
     * 166 5320 148 176tape 1264
     *
     *
     */

    public static void main(String... args) {

        /*
          * https://docs.google.com/spreadsheets/d/1zKXJPH7-HwzlicH3zPTKLC_NpEUBZXdVKZMOL9hayEg/edit?usp=sharing
         * example run
         * cd ~/workspace/2026-FRC-Rebuilt/build/libs$
         * java -cp 2026-FRC-Rebuilt.jar frc.robot.subsystems.LinearCalc
         */

        LinearCalc shotCalc = new LinearCalc();
        LinearCalc turretCalc = new LinearCalc();

        // Not in order but we figure it out... but good programming is to put it in
        // order

        /*
            132, 3520
            145, 3650
            156, 3815
            167, 4056
         */



        shotCalc.add(new LinearCalcRef(3470, 115, edu.wpi.first.units.Units.Inches));
        shotCalc.add(new LinearCalcRef(3520, 132, edu.wpi.first.units.Units.Inches));
        shotCalc.add(new LinearCalcRef(3650, 145, edu.wpi.first.units.Units.Inches));
        shotCalc.add(new LinearCalcRef(3815, 156, edu.wpi.first.units.Units.Inches));
        shotCalc.add(new LinearCalcRef(4056, 167, edu.wpi.first.units.Units.Inches));
        shotCalc.Calculate();

        turretCalc.add(new LinearCalcRef(0, 0, edu.wpi.first.units.Units.Degrees));
        turretCalc.add(new LinearCalcRef(250, 45, edu.wpi.first.units.Units.Degrees));
        turretCalc.add(new LinearCalcRef(779, 100, edu.wpi.first.units.Units.Degrees));
        turretCalc.add(new LinearCalcRef(812, 135, edu.wpi.first.units.Units.Degrees));
        turretCalc.add(new LinearCalcRef(846, 180, edu.wpi.first.units.Units.Degrees));
        turretCalc.add(new LinearCalcRef(812, 225, edu.wpi.first.units.Units.Degrees));
        turretCalc.add(new LinearCalcRef(779, 260, edu.wpi.first.units.Units.Degrees));
        turretCalc.add(new LinearCalcRef(250, 315, edu.wpi.first.units.Units.Degrees));
        turretCalc.add(new LinearCalcRef(0, 360, edu.wpi.first.units.Units.Degrees));
        turretCalc.Calculate();

        shotCalc.setMinimumRPM(1200);
        // LC.setMaximumRPM(5000);
        // LC.unsetMaximumRPM();
        // LC.unsetMinimumRPM();

        // You must call calculate to find the segments correctly and then they will be
        // used to
        // do the calculations

        System.out.printf("\n\n\n");
        shotCalc.printReferences();
        System.out.printf("\n\n\n");
        shotCalc.printSections();
        System.out.printf("\n\n\n");


        System.out.printf("    Dist :");
        System.out.printf(" 0 DEG  :");
        System.out.printf(" 45 DEG :");
        System.out.printf(" 90 DEG :");
        System.out.printf(" 135 DEG:");
        System.out.printf(" 180 DEG:");
        System.out.printf(" 270 DEG:");
        System.out.printf("   REF");
        System.out.printf("\n");



        for (int i = (int) 60; i <= (int) (shotCalc.getLongDistance() + 10); i++) {
            System.out.printf(" %6.2f ", (double) i);
            System.out.printf(" : %.1f", shotCalc.getRPM(i) + turretCalc.getRPM(0));
            System.out.printf(" : %.1f", shotCalc.getRPM(i) + turretCalc.getRPM(45));
            System.out.printf(" : %.1f", shotCalc.getRPM(i) + turretCalc.getRPM(90));
            System.out.printf(" : %.1f", shotCalc.getRPM(i) + turretCalc.getRPM(135));
            System.out.printf(" : %.1f", shotCalc.getRPM(i) + turretCalc.getRPM(180));
            System.out.printf(" : %.1f", shotCalc.getRPM(i) + turretCalc.getRPM(270));
            if (shotCalc.isReferenceDist(i)) {
                System.out.printf(" : %.1f", shotCalc.getRPM(i) + turretCalc.getRPM(0));
            }
            System.out.printf("\n");
        }

        // Calculating the Turret Curve
        System.out.println("\n\n");
        System.out.println("Turret Curve:\n\n");
        System.out.println("\tAssuming that all angles over 90 are the same as 180 degrees.\n\n\n");
        System.out.println("Turret Angle : Added RPM");

        System.out.printf("\n\n\n");
        turretCalc.printReferences();
        System.out.printf("\n\n\n");
        turretCalc.printSections();
        System.out.printf("\n\n\n");


        System.out.printf(" DEG  :");
        System.out.printf(" RPM+ :");
        System.out.printf(" Ref");
        System.out.printf("\n");



        for (int i = (int) 0; i <= 360; i+=5) {
            System.out.printf(" %4.2f ", (double) i);
            System.out.printf(" : %.1f", turretCalc.getRPM(i));
            if (turretCalc.isReferenceDist(i)) {
                System.out.printf(" : %.1f", turretCalc.getRPM(i) + turretCalc.getRPM(0));
            }
            System.out.printf("\n");
        }


    }
}