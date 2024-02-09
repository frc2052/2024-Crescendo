package frc.robot.util;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.states.RobotState;

public class RobotStateEstimator {
    static RobotStateEstimator INSTANCE;
    private RobotState robotState = RobotState.getInstance();

    private SwerveDrivePoseEstimator poseEstimator;

    /**
     * RobotStateEstimator uses SwerveDrivePoseEstimator to estimate the pose of the robot, field relative.
     */
    public static RobotStateEstimator getInstance(){
        if (INSTANCE == null) {
            INSTANCE = new RobotStateEstimator();
        }

        return INSTANCE;
    }

    private RobotStateEstimator() {}

    /**
     * Update the SwerveDrivePoseEstimator with values from RobotState
     */
    public void updateRobotPoseEstimator() {
        if (!robotState.hasValidSwerveState()) {
            return;
        }

        if(poseEstimator == null){
            poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Drivetrain.kinematics, 
                robotState.getRotation2d(), 
                robotState.getModulePositions(),
                new Pose2d()
            );
        }

        poseEstimator.setVisionMeasurementStdDevs(null);

        poseEstimator.addVisionMeasurement(
            robotState.getVisionPose3d().toPose2d(),
            robotState.getVisionDetectionTime()
        );
        
        poseEstimator.update(
            robotState.getRotation2d(), 
            robotState.getModulePositions()
        );

        robotState.updateRobotPose(poseEstimator.getEstimatedPosition()); 
    }

    /**
     * Reset the position of SwerveDrivePoseEstimator and set the NavX Offset
     */
    public void resetOdometry(Pose2d pose){
        robotState.resetInitialPose(pose);

        poseEstimator.resetPosition(
            robotState.getRotation2d(), 
            robotState.getModulePositions(),
            pose
        );
    }
}
