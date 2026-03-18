package frc.robot.subsystems;

public class LTurretCalc {

    double baseRPM = 866;

    LTurretCalc() {

    }

    public double getSinRPM(double degrees) {
        return Math.sin(Math.toRadians(degrees / 2)) * baseRPM;
    }

    public double getCosRPM(double degrees) {
        return ((1 - Math.cos(Math.toRadians(degrees))) / 2) * baseRPM;
    }

    public double getSinSinRPM(double degrees) {

        // 1. Convert degrees to radians
        double radians = Math.toRadians(degrees);

        // 2. Use sin(theta/2) so the peak (1.0) happens at 180 degrees
        double base = Math.abs(Math.sin(radians / 2.0));

        // 3. Raising to the power of 6.5 matches your "70 RPM at 90 deg" requirement
        // y = 862 * (sin(theta/2))^6.5

        return this.baseRPM * Math.pow(base, 6.5);

    }

    public void setBaseRPM(double base) {
        this.baseRPM = base;
    }



    public static void main(String... args) {

        LTurretCalc TC = new LTurretCalc();
        TC.setBaseRPM(900);

        for (int i = (int) 0; i < 360 + 1; i += 5) {
            System.out.printf(" %4.2f     : %.2f%n", (double) i, TC.getSinSinRPM(i));
        }
    }
}
