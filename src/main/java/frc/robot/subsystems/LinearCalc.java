package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
//import static edu.wpi.first.units.Units.Centermeter;



public class LinearCalc {
    private double m = 0.0;
    private double b = 0.0;
    private LinearCalcRef short_shot;
    private LinearCalcRef long_shot;
    
    


    public LinearCalc(LinearCalcRef shortShot, LinearCalcRef longShot) {

        // make sure both are in the same units.   
        
        if (shortShot.unitOfMeasure.toString() ==  shortShot.unitOfMeasure.toString()) {
            this.short_shot = shortShot;
            this.long_shot = longShot;

            // Note:  You must calculate the slope (m) first  
            calcM();
            calcY();
        }

    }

    private void calcM () {
        this.m = (long_shot.RPM - short_shot.RPM ) / (long_shot.Distance - short_shot.Distance);
    }

    private void calcY () {
        // y = mx + b
        // y -mx = b
        this.b = this.short_shot.RPM - (this.m * this.short_shot.Distance);
    }

    public double getRPM (double distance) {
       
        // y = mx + b
        // RPM = (this.m * distance) + this.b;
        return (this.m * distance) + this.b;
    }


public static void main(String... args) {
    

        // https://docs.google.com/spreadsheets/d/1wLWtShgRIa7HTyyL5FNpmJXy-W9DC3bK6cpnDJLcKoI/edit?usp=sharing 
        // example run
        // 2026-FRC-Rebuilt\build\libs> java -cp .\2026-FRC-Rebuilt.jar frc.robot.subsystems.LinearCalc

        LinearCalcRef shortShot = new LinearCalcRef(3250, 9.25, edu.wpi.first.units.Units.Feet);
     LinearCalcRef longShot = new LinearCalcRef(3700, 10.5, edu.wpi.first.units.Units.Feet);
        LinearCalc LC = new LinearCalc(shortShot, longShot);
        System.out.println("Distance :   RPM ");
        for (int i = (int) shortShot.Distance; i <= (int) longShot.Distance; i++) {
            
            System.out.printf(" %d     : %.2f%n", i, LC.getRPM(i));

        }
    }
}