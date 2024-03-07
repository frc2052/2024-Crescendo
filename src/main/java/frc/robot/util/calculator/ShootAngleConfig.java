package frc.robot.util.calculator;

public class ShootAngleConfig {
    private final double distancecm;
    private final double angleDegrees;
    private final double shooterSpeedVelocityRPS;

    public ShootAngleConfig(double distancecm, double angleDegrees, double shooterSpeedVelocity) {
        this.distancecm = distancecm;
        this.angleDegrees = angleDegrees;
        this.shooterSpeedVelocityRPS = shooterSpeedVelocity;
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

    public double getShooterSpeedVelocityRPS(){
        return shooterSpeedVelocityRPS;
    }
}
