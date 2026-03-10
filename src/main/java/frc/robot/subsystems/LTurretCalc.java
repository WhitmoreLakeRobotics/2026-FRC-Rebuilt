package frc.robot.subsystems;

public class LTurretCalc {

    double baseRPM = 0.0;

    LTurretCalc() {

    }

    public double getSinRPM(double degrees) {
        return Math.sin(Math.toRadians(degrees /2)) * baseRPM;
    }


    public void setBaseRPM(double base) {
        this.baseRPM = base;
    }

    public static void main(String... args) {

        LTurretCalc TC = new LTurretCalc();
        TC.setBaseRPM(50);

        for (int i = (int) 0; i < 360 + 1; i++) {
            System.out.printf(" %4.2f     : %.2f%n", (double)i, TC.getSinRPM(i));
        }
    }
}
