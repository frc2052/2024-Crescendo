package frc.robot.util.calculator;

import frc.robot.Constants;

public class ShootAngleConfig {
    private final double distancecm;
    private final double angleDegrees;
    private final double upperShooterVelocityPCT;
    private final double lowerShooterVelocityPCT;

    public ShootAngleConfig(double distancecm, double angleDegrees, double upperShooterSpeedPCT, double lowerShooterSpeedPCT) {
        this.distancecm = distancecm;
        this.angleDegrees = angleDegrees;
        this.upperShooterVelocityPCT = upperShooterSpeedPCT;
        this.lowerShooterVelocityPCT = lowerShooterSpeedPCT;
    }

    public double getDistanceCentimeters() {
        return this.distancecm;
    }

    public double getAngleDegrees() {
        return angleDegrees;
    }

    public double getAngleRadians() {
        return Math.toRadians(angleDegrees);
    }

    public double getUpperShooterSpeedVelocityRPS(){
        return (upperShooterVelocityPCT);
    }

    public double getLowerShooterSpeedVelocityRPS(){
        return (lowerShooterVelocityPCT);
    }
}
