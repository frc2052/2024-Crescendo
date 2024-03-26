package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.io.Dashboard;

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
        
        if (!(robotState.getChassisSpeeds().vxMetersPerSecond > 0.5) && !(robotState.getChassisSpeeds().vxMetersPerSecond > 0.5) && !(robotState.getChassisSpeeds().omegaRadiansPerSecond > 0.5)){            
            if(robotState.getVisionPose3d().isPresent()){
                Pose2d visionPose = robotState.getVisionPose3d().get().toPose2d();
                // square the x distance in meters and multiply by 0.05 to get how much we trust the vision
                double xyStds = 0.05 * Math.pow(robotState.getSpeakerLocation().getDistance(visionPose.getTranslation()), 2);
                // System.out.println("STDS " + xyStds);
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, 99999999));

                poseEstimator.addVisionMeasurement(
                    visionPose,
                    robotState.getVisionDetectionTime()
                );
            }
            
            Dashboard.getInstance().putData("VISION IN USE?", true);
        } else {
            Dashboard.getInstance().putData("VISION IN USE?", false);
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
