package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team2052.lib.photonvision.EstimatedRobotPose;
import com.team2052.lib.photonvision.PhotonCamera;
import com.team2052.lib.photonvision.PhotonPoseEstimator;
import com.team2052.lib.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera0, camera0.getRobotToCamera());
    private int speakertag;
    private double speakerTagYaw;
    private Timer timer;
    private double lastUpdatedTime;
    private boolean isSeeingTag;

    //private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy., camera0, camera0.getRobotToCamera());
    //private PhotonCamera camera1 = new PhotonCamera(Constants.PhotonCamera2.CAMERA_NAME, Constants.PhotonCamera2.ROBOT_TO_CAMERA_METERS);
  
    public static AprilTagSubsystem getInstance(){
        if (INSTANCE == null){
            INSTANCE = new AprilTagSubsystem();
        } 
        
        return INSTANCE;
    }

    private AprilTagSubsystem() {
        cameras.add(camera0);
        // cameras.add(camera1);

        timer = new Timer();

        if (!RobotState.getInstance().isRedAlliance()){
            speakertag = 7; //blue
        } else {
            speakertag = 4;//red
        }
        timer.start();
        isSeeingTag = false;
        
    }

    @Override
    public void periodic() {
        
        boolean sawTag = false;
        for(int i = 0; i < cameras.size(); i++){
            PhotonCamera camera = cameras.get(i);
            var result = camera.getLatestResult();
            var targets = result.getTargets();

            boolean hasTargets = result.hasTargets();
            if (hasTargets){
                // PhotonTrackedTarget target7 = null;
                // for(i = 0; i < result.targets.size(); i++){
                //     if(result.targets.get(i).getFiducialId() == 7){
                //         target7 = result.targets.get(i);
                //         break;
                //     }
                // }
                // if(target7 != null) {
                //     photonPoseEstimator.update();
                // }
                
                Optional<EstimatedRobotPose> poseUpdate = photonPoseEstimator.update();
                if (poseUpdate.isPresent()) {
                    estimatedPose = poseUpdate.get();
                    robotState.addAprilTagVisionUpdate(estimatedPose);
                }
                for (int j = 0; j < targets.size(); j++) {
                    System.out.println("IDs : " + targets.get(j).getFiducialId());
                    System.out.println("SpeakerTag : " + speakertag);
                    if (targets.get(j).getFiducialId() == speakertag) {
                        speakerTagYaw = targets.get(j).getYaw();
                        double lastUpdatedTime = timer.get();
                        sawTag = true;
                        System.out.println("Saw tag 7");
                    }
                }
                
            }
        }
        if (sawTag) {
            sawTag = false;
            isSeeingTag = true;
        } else {
            isSeeingTag = false;
        }

        Logger.recordOutput("Is Seeing Speaker April Tag", isSeeingTag);
        Logger.recordOutput("SpeakerTag Yaw", speakerTagYaw);
    }

    public Optional<Double> getYaw() {
        if (timer.get() - lastUpdatedTime <= Constants.AprilTags.APRILTAG_TIMEOUT) {
            return Optional.of(speakerTagYaw);
        } else {
            return Optional.empty();
        }
    }

    public boolean isSeeingSpeakerTag() {
        return isSeeingTag;
    }
}
