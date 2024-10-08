package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
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

    public void onRobotPeriodic() {
        if (DriverStation.getMatchTime() >= 130 && !hasRunTwentySeconds) {
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
