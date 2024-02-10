package frc.robot.commands.music;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MusicPlayerSubsystem;

public class PlayTeleopJingleCommand extends Command{
    private final MusicPlayerSubsystem player;

    public PlayTeleopJingleCommand(MusicPlayerSubsystem player) {
        this.player = player;
        addRequirements(player);
    }

    @Override
    public void initialize() {
        player.stopMusic();
        player.loadMusic("TeleopJingle.chrp");
        player.playLoadedTrack();
    }

    @Override
    public void end(boolean interrupted) {
        player.stopMusic();
    }

}
