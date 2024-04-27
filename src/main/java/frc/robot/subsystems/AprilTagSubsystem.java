package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;

import com.team2052.lib.photonvision.VisionUpdate;
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

    private List<VisionUpdate> estimatedPoses = new ArrayList<VisionUpdate> ();

    private RobotState robotState = RobotState.getInstance();
    private List<PhotonCamera> cameras = new ArrayList<PhotonCamera> ();
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // private PhotonCamera camera0 = new PhotonCamera(Constants.PhotonCamera0.CAMERA_NAME, Constants.PhotonCamera0.ROBOT_TO_CAMERA_METERS, aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    private PhotonCamera camera1 = new PhotonCamera(Constants.PhotonCamera1.CAMERA_NAME, Constants.PhotonCamera1.ROBOT_TO_CAMERA_METERS, aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    // private PhotonCamera camera2 = new PhotonCamera(Constants.PhotonCamera2.CAMERA_NAME, Constants.PhotonCamera2.ROBOT_TO_CAMERA_METERS, aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    // private PhotonCamera camera3 = new PhotonCamera(Constants.PhotonCamera3.CAMERA_NAME, Constants.PhotonCamera3.ROBOT_TO_CAMERA_METERS, aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  
    public static AprilTagSubsystem getInstance(){
        if (INSTANCE == null){
            INSTANCE = new AprilTagSubsystem();
        } 
        
        return INSTANCE;
    }

    private AprilTagSubsystem() {
        // cameras.add(camera0);
        cameras.add(camera1);
        // cameras.add(camera2);
        // cameras.add(camera3);
    }

    @Override
    public void periodic() {
        for(int i = 0; i < cameras.size(); i++){
            PhotonCamera camera = cameras.get(i);
            PhotonPipelineResult result = camera.getLatestResult();
            PhotonPoseEstimator poseEstimator = camera.getPoseEstimator();

            boolean hasTargets = result.hasTargets();
            if (hasTargets){
                Optional<VisionUpdate> poseUpdate = poseEstimator.update();
                if (poseUpdate.isPresent()) {
                    estimatedPoses.add(poseUpdate.get());
                }
            }
        }

        
        robotState.addAprilTagVisionUpdates(estimatedPoses);
    }

    public Optional<Double> getYaw() {
        return Optional.empty();
    }

    public boolean isSeeingSpeakerTag() {
        return false;
    }
}
