package frc.robot.util.calculator;

import java.util.ArrayList;
import java.util.List;
import frc.robot.Constants;
import frc.robot.commands.auto.commands.shamper.ShootAutoLowCommand;

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
        // shootAngleLookup.add(new ShootAngleConfig(100, 55, 0.7 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(150, 53, 0.7 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(200, 47, 0.8 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(250, 41, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(300, 38, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(350, 33, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(400, 32, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(450, 28, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(500, 27, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(550, 26, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(600, 25, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));

        // shootAngleLookup.add(new ShootAngleConfig(130, 52, 0.5 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(200, 44, 0.51  * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(250, 39, 0.55 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(300, 36.5, 0.6 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(350, 32.5, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(400, 29.25, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(450, 28, 0.85 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(500, 26.5, 0.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));

        shootAngleLookup.clear();

        // shootAngleLookup.add(new ShootAngleConfig(136, 54, 0.45 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.55 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(150, 51, 0.55 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(170, 47.5, 0.55 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(199, 44.75, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(223, 42.5, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(249, 39, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(283, 37.75, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(303, 36.25, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(327, 35.5, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(349, 34.25, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(405, 32.5, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(451, 29, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.85 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(500, 27.75, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.85 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        // shootAngleLookup.add(new ShootAngleConfig(555, 26.75, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.85 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));

        shootAngleLookup.add(new ShootAngleConfig(136, 54, 0.85 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.90 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(150, 51, 0.55 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(170, 47.5, 0.55 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(199, 44.75, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(223, 41.5, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(249, 38, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(283, 36.75, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(303, 35.25, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(327, 34.5, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(349, 32.5, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(375, 31.5, 0.65 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(405, 31, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.85 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(451, 27.5, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.85 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(500, 26.75, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.85 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
        shootAngleLookup.add(new ShootAngleConfig(555, 25.75, 0.75 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.85 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS));
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
                break;
            } else {
                upperDistanceConfig = shootAngleLookup.get(i);
            }
        }

        // Returns a default shooter configuration if either of the bounds are null
        if (lowerDistanceConfig == null) {
            System.out.println("No Lower Config");
            return shootAngleLookup.get(shootAngleLookup.size() - 1);
        } else if (upperDistanceConfig == null){
            System.out.println("No Upper Config");
            return shootAngleLookup.get(0);
        }

        // deltaInches is the difference between the lower and upper pre-measured inches values.
        double deltacm = upperDistanceConfig.getDistanceCentimeters() - lowerDistanceConfig.getDistanceCentimeters();
        // offsetInches is the difference between our actual current distance inches and the upper distance inches.
        double offsetcm = upperDistanceConfig.getDistanceCentimeters() - distancecm;

        double pct = offsetcm / deltacm;

        // Calculate the difference between the two top motor velocities from the lookup table entries.
        double deltaTopMotorVelocityTicksPerSecond = upperDistanceConfig.getUpperShooterSpeedVelocityRPS() - lowerDistanceConfig.getUpperShooterSpeedVelocityRPS();
        // Multiple the difference of top motor velocities by the percentage of offsetInches and deltaInches. 
        double offsetTopMotorVelocityTicksPerSecond = deltaTopMotorVelocityTicksPerSecond * pct;

        // Calculate the difference between the two bottom motor velocities from the lookup table entries.
        double deltaBottomMotorVelocityTicksPerSecond = upperDistanceConfig.getLowerShooterSpeedVelocityRPS() - lowerDistanceConfig.getLowerShooterSpeedVelocityRPS();
        // Multiple the difference of bottom motor velocities by the percentage of offsetInches and deltaInches. 
        double offsetBottomMotorVelocityTicksPerSecond = deltaBottomMotorVelocityTicksPerSecond * pct;
        
        double deltaAngle = upperDistanceConfig.getAngleDegrees() - lowerDistanceConfig.getAngleDegrees();

        double offsetAngle = deltaAngle * pct;

        return new ShootAngleConfig(
            distancecm,
            // Add the final offsets to our lower distance so the bottom shooter configuration can safely be assumed.
            upperDistanceConfig.getAngleDegrees() - offsetAngle,
            // Add the final offsets to our lower distance so the top shooter configuration can safely be assumed.
            upperDistanceConfig.getUpperShooterSpeedVelocityRPS() - offsetTopMotorVelocityTicksPerSecond,
            upperDistanceConfig.getLowerShooterSpeedVelocityRPS() - offsetBottomMotorVelocityTicksPerSecond
        );
    }
}
