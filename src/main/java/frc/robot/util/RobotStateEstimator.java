package frc.robot.util;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

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
        
        if(robotState.getVisionPose3d().isPresent()){
            Pose2d visionPose = robotState.getVisionPose3d().get().toPose2d();
            // if vision pose outside of field, it's fake, it's lying
            if(visionPose.getX() > 0 && visionPose.getX() < Units.inchesToMeters(651.157) && visionPose.getY() > 0 && visionPose.getY() < Units.feetToMeters(27)){
                double distanceToSpeaker = robotState.getSpeakerLocation().getDistance(robotState.getRobotPose().getTranslation());
                double xyPower = 1;
                double rotStds = 99999999;
                // System.out.println("STDS " + xyStds);
                List<PhotonTrackedTarget> targets = robotState.getActiveTargets();
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

                // if we are close enough and see both speaker tags, make the pose estimator trust rotation from tags more
                if(distanceToSpeaker < 2.5 && foundTag1 && foundTag2) {
                    rotStds = 0.2;
                }

                if (foundTag1 || foundTag2){
                    xyPower = 1;
                }

                double xyStds = 0.03 * Math.pow(distanceToSpeaker, xyPower);
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, rotStds));

                poseEstimator.addVisionMeasurement(
                    visionPose,
                    robotState.getVisionDetectionTime()
                );
            }
        }

        poseEstimator.update(
            robotState.getGyroRotation(), 
            robotState.getModulePositions()
        );

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
