package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.commands.music.PlayActivationJingleCommand;
import frc.robot.commands.music.PlayTeleopJingleCommand;
import frc.robot.commands.music.PlayTwentySecondsLeftCommand;
import frc.robot.subsystems.MusicPlayerSubsystem;

public class RobotStatusCommunicator {
    MusicPlayerSubsystem musicPlayer;
    private boolean hasRunTwentySeconds;
    
    public RobotStatusCommunicator(MusicPlayerSubsystem musicPlayer) {
        this.musicPlayer = musicPlayer;
        hasRunTwentySeconds = false;
    }

    public void onRobotInitiation() {
        if (RobotContainer.musicOn) {
            new PlayActivationJingleCommand(musicPlayer);
        }
    }

    public void onRobotDisable() {
    //BE VERY CARFUL WHAT IS PUT IN HERE BECAUSE IT WILL BE DONE AFTER THE ROBOT IS DISABLED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }

    public void onRobotPeriodic() {

        if (DriverStation.getMatchTime() >= 180 && !hasRunTwentySeconds) {
            onTwentySecondsLeft();
            hasRunTwentySeconds = true;
        }
    }

    public void onRobotTeleop() {
        if (RobotContainer.musicOn) {
            new PlayTeleopJingleCommand(musicPlayer);
        }
    }

    public void onTwentySecondsLeft() {
        if (RobotContainer.musicOn) {
            new PlayTwentySecondsLeftCommand(musicPlayer);
        }
    }




    
}
