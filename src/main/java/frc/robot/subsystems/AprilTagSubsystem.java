package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.team2052.lib.photonvision.EstimatedRobotPose;
import com.team2052.lib.photonvision.PhotonCamera;
import com.team2052.lib.photonvision.PhotonPoseEstimator;
import com.team2052.lib.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class AprilTagSubsystem extends SubsystemBase{
    public static AprilTagSubsystem INSTANCE;

    private EstimatedRobotPose estimatedPose;

    private RobotState robotState = RobotState.getInstance();
    private List<PhotonCamera> cameras = new ArrayList<PhotonCamera> ();
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonCamera camera0 = new PhotonCamera(Constants.PhotonCamera1.CAMERA_NAME, Constants.PhotonCamera1.ROBOT_TO_CAMERA_METERS);
    //private PhotonCamera camera1 = new PhotonCamera(Constants.PhotonCamera2.CAMERA_NAME, Constants.PhotonCamera2.ROBOT_TO_CAMERA_METERS);
  
    public static AprilTagSubsystem getInstance(){
        if (INSTANCE == null){
            INSTANCE = new AprilTagSubsystem();
        } 
        
        return INSTANCE;
    }

    private AprilTagSubsystem() {
        cameras.add(camera0);
        //cameras.add(camera1);
    }

    @Override
    public void periodic() {
        for(int i = 0; i < cameras.size(); i++){
            PhotonCamera camera = cameras.get(i);
            var result = camera.getLatestResult();
            boolean hasTargets = result.hasTargets();
            if (hasTargets){
                PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, camera.getRobotToCamera());
                Optional<EstimatedRobotPose> poseUpdate = photonPoseEstimator.update();
                if (poseUpdate.isPresent()) {
                    estimatedPose = poseUpdate.get();
                    robotState.addAprilTagVisionUpdate(estimatedPose);
                }
            }
        }
    }
}
