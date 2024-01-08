package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.team2052.lib.PiCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;

public class AprilTagSubsystem{
    public static AprilTagSubsystem INSTANCE;

    private RobotState robotState = RobotState.getInstance();
    private List<PiCamera> cameras = new ArrayList<PiCamera> ();
    //private PiCamera[] cameras = new PiCamera[2];
  
    public static AprilTagSubsystem getInstance(){
        if (INSTANCE == null){
            INSTANCE = new AprilTagSubsystem();
        } 
        
        return INSTANCE;
    }

    private AprilTagSubsystem() {
        cameras.add(new PiCamera("PiCamera1", Constants.PiCamera1.PI_CAMERA_POSITION_METERS));
    }

    public void update() {
        Translation2d totalTranslation = new Translation2d();
        double totalRotationRadians = 0;
        int totalNumCameras = 0;

        for(int i = 0; i < cameras.size(); i++){
            PiCamera camera = cameras.get(i);
            if(camera.isValid()){
                totalTranslation = totalTranslation.plus(camera.getEstimatedPosition().getTranslation());
                totalRotationRadians += camera.getEstimatedPosition().getRotation().getRadians();
                totalNumCameras++;
            } else {
                System.out.println(camera.getName() + " IS INVALID");
            }
        }

        Translation2d averageTranslation2d = totalTranslation.div(totalNumCameras);
        Rotation2d averageRotation2d = new Rotation2d(totalRotationRadians / totalNumCameras);
        robotState.addVisionPose2dUpdate(new Pose2d(averageTranslation2d, averageRotation2d));
    }
}
