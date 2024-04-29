package frc.robot.util;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.team2052.lib.photonvision.VisionUpdate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
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
                robotState.getGyroRotation(), 
                robotState.getModulePositions(),
                robotState.getInitialPose(),
                Constants.Drivetrain.ODOMETRY_STDDEV,
                Constants.Vision.VISION_STDDEV
            );
        }
        
        /*
         *  Odometry updates
         */
        if(!robotState.getCollisionDetected()){
            poseEstimator.update(
                robotState.getGyroRotation(), 
                robotState.getModulePositions()
            );
        }

        /*
         * Vision updates
         */
        
        for(VisionUpdate visionUpdate : robotState.getVisionUpdates()){
            Pose2d visionPose = visionUpdate.estimatedPose.toPose2d();
            double xyStds = 0.25;
            double rotStds = 99999999;

            List<PhotonTrackedTarget> targets = visionUpdate.targetsUsed;
            boolean foundTag1 = false;
            boolean foundTag2 = false;

            // check list of seen targets and if we see both speaker tags
            for (PhotonTrackedTarget target : targets) {
                if(target.getFiducialId() == 3 || target.getFiducialId() == 7){
                    foundTag1 = true;
                } else if (target.getFiducialId() == 4 || target.getFiducialId() == 8){
                    foundTag2 = true;
                }
            }

            // if we see both speaker tags, make the pose estimator trust rotation from tags more
            if(foundTag1 && foundTag2) {
                rotStds = 0.2;
            }

            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, rotStds));

            poseEstimator.addVisionMeasurement(
                visionPose,
                visionUpdate.timestampSeconds
            );
        }

        robotState.updateRobotPose(poseEstimator.getEstimatedPosition()); 
    }

    /**
     * Reset the position of SwerveDrivePoseEstimator and set the NavX Offset
     */
    public void resetOdometry(Pose2d pose){
        //robotState.resetInitialPose(pose);
        if(poseEstimator != null){
            poseEstimator.resetPosition(
                robotState.getGyroRotation(), 
                robotState.getModulePositions(),
                pose
            );
        }

    }
}
