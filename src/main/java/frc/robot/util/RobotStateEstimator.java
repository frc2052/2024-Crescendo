package frc.robot.util;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RobotState;

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
                robotState.getRotation2dRaw(), 
                robotState.getModulePositions(),
                robotState.getInitialPose(),
                Constants.Drivetrain.ODOMETRY_STDDEV,
                Constants.Vision.VISION_STDDEV
            );
        }
        
        if(DriverStation.isTeleop()){
            if(robotState.getVisionPose3d().isPresent()){
                Pose2d visionPose = robotState.getVisionPose3d().get().toPose2d();//new Pose2d(robotState.getVisionPose3d().get().getTranslation().toTranslation2d(), robotState.getRotation2d180());
                poseEstimator.addVisionMeasurement(
                    visionPose,
                    robotState.getVisionDetectionTime()
                );
            }
        }

        poseEstimator.update(
            robotState.getRotation2dRaw(), 
            robotState.getModulePositions()
        );

        robotState.updateRobotPose(poseEstimator.getEstimatedPosition()); 
    }

    /**
     * Reset the position of SwerveDrivePoseEstimator and set the NavX Offset
     */
    public void resetOdometry(Pose2d pose){
        //robotState.resetInitialPose(pose);

        poseEstimator.resetPosition(
            robotState.getRotation2d360(), 
            robotState.getModulePositions(),
            pose
        );
    }
}
