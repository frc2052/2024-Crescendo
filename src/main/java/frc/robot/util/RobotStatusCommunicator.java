package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.music.PlayActivationJingleCommand;
import frc.robot.commands.music.PlayTeleopJingleCommand;
import frc.robot.commands.music.PlayTwentySecondsLeftCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.MusicPlayerSubsystem;
// import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.RobotContainer;
// import frc.robot.commands.music.PlayActivationJingleCommand;
// import frc.robot.commands.music.PlayTeleopJingleCommand;
// import frc.robot.commands.music.PlayTwentySecondsLeftCommand;
// import frc.robot.subsystems.MusicPlayerSubsystem;

public class RobotStatusCommunicator {
    private RobotState robotState = RobotState.getInstance();
    private MusicPlayerSubsystem musicPlayer;
    private boolean hasRunTwentySeconds;
    
    public RobotStatusCommunicator(MusicPlayerSubsystem musicPlayer) {
        this.musicPlayer = musicPlayer;
        hasRunTwentySeconds = false;
    }

    public void onRobotInitiation() {
        if (robotState.getMusicEnableStatus()) {
            new PlayActivationJingleCommand(musicPlayer);
        }
    }

//     public void onRobotDisable() {
//     //BE VERY CARFUL WHAT IS PUT IN HERE BECAUSE IT WILL BE DONE AFTER THE ROBOT IS DISABLED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//     }

    public void onRobotPeriodic() {
        if (DriverStation.getMatchTime() >= 180 && !hasRunTwentySeconds) {
            onTwentySecondsLeft();
            hasRunTwentySeconds = true;
        }
    }

    public void onRobotTeleop() {
        if (robotState.getMusicEnableStatus()) {
            new PlayTeleopJingleCommand(musicPlayer);
        }
    }

    public void onTwentySecondsLeft() {
        if (robotState.getMusicEnableStatus()) {
            new PlayTwentySecondsLeftCommand(musicPlayer);
        }
    }




    
}
