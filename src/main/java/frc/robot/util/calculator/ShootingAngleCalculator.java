package frc.robot.util.calculator;

import java.util.ArrayList;
import java.util.List;
import frc.robot.Constants;

public class ShootingAngleCalculator {
    private static ShootingAngleCalculator INSTANCE;

    private List<ShootAngleConfig> shootAngleLookup;

    public static ShootingAngleCalculator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShootingAngleCalculator();
        }
        return INSTANCE;
    }

    public ShootingAngleCalculator(){
        shootAngleLookup = new ArrayList<ShootAngleConfig>();
        
        setupShootAngleLookup();
    }

    public void setupShootAngleLookup(){
        shootAngleLookup.add(new ShootAngleConfig(100, 55, 0.7 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(150, 53, 0.7 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(200, 47, 0.8 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(250, 41, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(300, 38, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(350, 33, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(400, 32, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(450, 28, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(500, 27, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(550, 26, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(600, 25, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
    }

    public ShootAngleConfig getShooterConfig(double distanceMeters) {
        // Lower bound of the estimated shooter configuration given the distance from the target.
        ShootAngleConfig lowerDistanceConfig = null;
        // Upper bound of the estimated shooter configuration given the distance from the target.
        ShootAngleConfig upperDistanceConfig = null;

        // convert current pose meters to centimeters
        double distancecm = distanceMeters * 100;
        
        for(int i = 0; i < shootAngleLookup.size(); i++){
            if (distancecm < shootAngleLookup.get(i).getDistanceCentimeters()){
                lowerDistanceConfig = shootAngleLookup.get(i);
                // System.out.println("Dictionary " + i);
                break;
            } else {
                upperDistanceConfig = shootAngleLookup.get(i);
            }
        }

        // Returns a default shooter configuration if either of the bounds are null
        if (lowerDistanceConfig == null) {
            System.out.println("No Lower Config");
            return shootAngleLookup.get(0);
        } else if (upperDistanceConfig == null){
            System.out.println("No Upper Config");
            return shootAngleLookup.get(shootAngleLookup.size() - 1);
        }

        // deltaInches is the difference between the lower and upper pre-measured inches values.
        double deltacm = upperDistanceConfig.getDistanceCentimeters() - lowerDistanceConfig.getDistanceCentimeters();
        // offsetInches is the difference between our actual current distance inches and the upper distance inches.
        double offsetcm = upperDistanceConfig.getDistanceCentimeters() - distancecm;

        double pct = offsetcm / deltacm;

        // Calculate the difference between the two top motor velocities from the lookup table entries.
        double deltaTopMotorVelocityTicksPerSecond = upperDistanceConfig.getShooterSpeedVelocityRPS() - lowerDistanceConfig.getShooterSpeedVelocityRPS();
        // Multiple the difference of top motor velocities by the percentage of offsetInches and deltaInches. 
        double offsetTopMotorVelocityTicksPerSecond = deltaTopMotorVelocityTicksPerSecond * pct;

        // Calculate the difference between the two bottom motor velocities from the lookup table entries.
        double deltaBottomMotorVelocityTicksPerSecond = upperDistanceConfig.getShooterSpeedVelocityRPS() - lowerDistanceConfig.getShooterSpeedVelocityRPS();
        // Multiple the difference of bottom motor velocities by the percentage of offsetInches and deltaInches. 
        double offsetBottomMotorVelocityTicksPerSecond = deltaBottomMotorVelocityTicksPerSecond * pct;

        
        double deltaAngle = upperDistanceConfig.getAngleDegrees() - lowerDistanceConfig.getAngleDegrees();

        double offsetAngle = deltaAngle * pct;

        return new ShootAngleConfig(
            distancecm,
            // Add the final offsets to our lower distance so the bottom shooter cofniguration can safely be assumed.
            upperDistanceConfig.getAngleDegrees() - offsetAngle,
            // Add the final offsets to our lower distance so the top shooter cofniguration can safely be assumed.
            upperDistanceConfig.getShooterSpeedVelocityRPS() - offsetTopMotorVelocityTicksPerSecond
        );
    }
}
