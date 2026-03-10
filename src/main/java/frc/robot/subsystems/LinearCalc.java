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

    public void unsetMinimumRPM(){
        minRPMSet = false;
    }

    public void setMaximumRPM(double maxRPM){
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
        references.sort(Comparator.comparing(LinearCalcRef::getDistance));

        // iterate over the references. NOTE there must be more than 2 references

        if (references.size() < 2) {
            throw new ArrayIndexOutOfBoundsException(
                    "Not enough data:  Must provide atleast 2 LinearCalReferences. \n" +
                            "We have " + references.size() + ".");
        }

        for (int i = 0; i < references.size() - 1; i++) {
            sections.add(new LinearCalcSection(references.get(i), references.get(i + 1)));

        }

    }

    public double getRPM(double distance) {

        double rpm = 0.0;

        // If distance is less than short shot of first section then use first section
        if (distance < sections.get(0).getShortDistance()) {
            rpm = sections.get(0).getRPM(distance);
        }

        if (distance > sections.get(sections.size() - 1).getLongDistance()) {
            rpm = sections.get(sections.size() - 1).getRPM(distance);
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
            if (rpm > maximumRPM){
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

    public void printSections() {
        System.out.printf("Number of sections  %d\n", sections.size());
        for (int i = 0; i < sections.size(); i++) {
            System.out.printf("Short Distance %6.2f    RPM %6.2f\t", sections.get(i).getShortDistance(),
                    sections.get(i).getShortRPM());
            System.out.printf("Long Distance %6.2f     RPM %6.2f%n", sections.get(i).getLongDistance(),
                    sections.get(i).getLongRPM());
        }
    }

    public static void main(String... args) {

        /*
         * https://docs.google.com/spreadsheets/d/1zKXJPH7-HwzlicH3zPTKLC_NpEUBZXdVKZMOL9hayEg/edit?usp=sharing
         * example run
         * cd ~/workspace/2026-FRC-Rebuilt/build/libs$
         * java -cp 2026-FRC-Rebuilt.jar frc.robot.subsystems.LinearCalc
         *
         * Number of references 5
         * Distance 4.00 RPM 2003.00
         * Distance 5.20 RPM 2250.00
         * Distance 6.40 RPM 8000.00
         * Distance 8.00 RPM 1000.00
         * Distance 10.90 RPM 4700.00
         *
         *
         * Number of sections 4
         * Short Distance 4.00 RPM 2003.00 Long Distance 5.20 RPM 2250.00
         * Short Distance 5.20 RPM 2250.00 Long Distance 6.40 RPM 8000.00
         * Short Distance 6.40 RPM 8000.00 Long Distance 8.00 RPM 1000.00
         * Short Distance 8.00 RPM 1000.00 Long Distance 10.90 RPM 4700.00
         *
         *
         * 0 : 1179.67
         * 1 : 1385.50
         * 2 : 1591.33
         * 3 : 1797.17
         * 4 : 2003.00
         * 5 : 2208.83
         * 6 : 6083.33
         * 7 : 5375.00
         * 8 : 1000.00
         * 9 : 2275.86
         * 10 : 3551.72
         * 11 : 4827.59
         * 12 : 6103.45
         * 13 : 7379.31
         * 14 : 8655.17
         * 15 : 9931.03
         */
        LinearCalc LC = new LinearCalc();

        // Not in order but we figure it out... but good programming is to put it in
        // order
        LC.add(new LinearCalcRef(0, 0, Degree));
        LC.add(new LinearCalcRef(5, 45, Degree));
        LC.add(new LinearCalcRef(15, 90, Degree));
        LC.add(new LinearCalcRef(35, 135, Degree));
        LC.add(new LinearCalcRef(70, 180, Degree));
        LC.add(new LinearCalcRef(35, 225, Degree));
        LC.add(new LinearCalcRef(15, 270, Degree));
        LC.add(new LinearCalcRef(5, 315, Degree));
        LC.add(new LinearCalcRef(0, 360, Degree));

        LC.setMinimumRPM(0);
        //LC.setMaximumRPM(5000);
        //LC.unsetMaximumRPM();
        // LC.unsetMinimumRPM();

        // You must call calculate to find the segments correctly and then they will be
        // used to
        // do the calculations
        LC.Calculate();

        System.out.printf("\n\n\n");
        LC.printReferences();
        System.out.printf("\n\n\n");

        LC.printSections();

        System.out.printf("\n\n\n");

        for (int i = (int) 0; i <= (int) (LC.getLongDistance() + 1); i++) {
            // for (int j = (int) 0; j < 10; j++)
                //System.out.printf(" %4.2f     : %.2f%n", (double) (i + (j * .1)), LC.getRPM((double) (i + (j * .1))));
                System.out.printf(" %4.2f     : %.2f%n", (double) (i + (0 * .1)), LC.getRPM((double) (i + (0 * .1))));
            //}
        }

    }
}