package frc.robot.subsystems;

public class LinearCalcSection {

    LinearCalcRef short_shot;
    LinearCalcRef long_shot;
    double m = 0.0;
    double b = 0.0;

    LinearCalcSection (LinearCalcRef shortShot, LinearCalcRef longShot){
        this.short_shot = shortShot;
        this.long_shot = longShot;
        calcM();
        calcB();
    }


    private void calcM () {
        this.m = (long_shot.RPM - short_shot.RPM ) / (long_shot.Distance - short_shot.Distance);
    }

    private void calcB () {
        // y = mx + b
        // y - mx = b
        // b = y - mx

        this.b = this.short_shot.RPM - (this.m * this.short_shot.Distance);

    }

    public double getRPM (double distance) {

        // y = mx + b
        // RPM = (this.m * distance) + this.b;
        return (this.m * distance) + this.b;
    }

    public double getShortDistance (){
        return this.short_shot.getDistance();
    }

    public double getLongDistance (){
        return this.long_shot.getDistance();
    }

    public double getShortRPM(){
        return short_shot.getRPM();
    }

    public double getLongRPM (){
        return long_shot.getRPM();
    }


}