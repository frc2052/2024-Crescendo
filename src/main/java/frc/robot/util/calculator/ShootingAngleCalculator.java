package frc.robot.util.calculator;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

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
        shootAngleLookup.add(new ShootAngleConfig(90, 53, 1));
        shootAngleLookup.add(new ShootAngleConfig(200, 39, 1));
        shootAngleLookup.add(new ShootAngleConfig(300, 33, 1));
        shootAngleLookup.add(new ShootAngleConfig(400, 29, 1));
        shootAngleLookup.add(new ShootAngleConfig(500, 26.5, 1));
        shootAngleLookup.add(new ShootAngleConfig(600, 24.25, 1));
    }

    public ShootAngleConfig getShooterConfig(Pose2d robotPose) {
        // Lower bound of the estimated shooter configuration given the distance from the target.
        ShootAngleConfig lowerDistanceConfig = null;
        // Upper bound of the estimated shooter configuration given the distance from the target.
        ShootAngleConfig upperDistanceConfig = null;

        // convert current pose meters to centimeters
        double distanceX = robotPose.getX() / 100;
        
        for(int i = 0; i < shootAngleLookup.size(); i++){
            if (distanceX < shootAngleLookup.get(i).getDistanceCentimeters()){
                lowerDistanceConfig = shootAngleLookup.get(i);
            } else {
                upperDistanceConfig = shootAngleLookup.get(i);
                break;
            }
        }

        // Returns a default shooter configuration if either of the bounds are null
        if (lowerDistanceConfig == null) {
            return shootAngleLookup.get(0);
        } else if (upperDistanceConfig == null){
            return shootAngleLookup.get(shootAngleLookup.size());
        }

        // deltaInches is the difference between the lower and upper pre-measured inches values.
        double deltaInches = upperDistanceConfig.getDistanceCentimeters() - lowerDistanceConfig.getDistanceCentimeters();
        // offsetInches is the difference between our actual current distance inches and the upper distance inches.
        double offsetInches = upperDistanceConfig.getDistanceCentimeters() - distanceX;

        double pct = offsetInches / deltaInches;

        // Calculate the difference between the two top motor velocities from the lookup table entries.
        double deltaTopMotorVelocityTicksPerSecond = upperDistanceConfig.getShooterSpeedPercent() - lowerDistanceConfig.getShooterSpeedPercent();
        // Multiple the difference of top motor velocities by the percentage of offsetInches and deltaInches. 
        double offsetTopMotorVelocityTicksPerSecond = deltaTopMotorVelocityTicksPerSecond * pct;

        // Calculate the difference between the two bottom motor velocities from the lookup table entries.
        double deltaBottomMotorVelocityTicksPerSecond = upperDistanceConfig.getShooterSpeedPercent() - lowerDistanceConfig.getShooterSpeedPercent();
        // Multiple the difference of bottom motor velocities by the percentage of offsetInches and deltaInches. 
        double offsetBottomMotorVelocityTicksPerSecond = deltaBottomMotorVelocityTicksPerSecond * pct;

        return new ShootAngleConfig(
            distanceX,
            // Add the final offsets to our lower distance so the bottom shooter cofniguration can safely be assumed.
            upperDistanceConfig.getShooterSpeedPercent() - offsetBottomMotorVelocityTicksPerSecond,
            // Add the final offsets to our lower distance so the top shooter cofniguration can safely be assumed.
            upperDistanceConfig.getShooterSpeedPercent() - offsetTopMotorVelocityTicksPerSecond
        );
    }
}
