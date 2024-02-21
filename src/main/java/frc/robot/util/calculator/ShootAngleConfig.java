package frc.robot.util.calculator;

public class ShootAngleConfig {
    private final double distancecm;
    private final double angleDegrees;
    private final double shooterSpeedPCT;

    public ShootAngleConfig(double distancecm, double angleDegrees, double shooterSpeedPCT) {
        this.distancecm = distancecm;
        this.angleDegrees = angleDegrees;
        this.shooterSpeedPCT = shooterSpeedPCT;
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

    public double getShooterSpeedPercent(){
        return shooterSpeedPCT;
    }
}
