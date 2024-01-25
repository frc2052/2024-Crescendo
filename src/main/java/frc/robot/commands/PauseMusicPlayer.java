package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MusicPlayerSubsystem;

public class PauseMusicPlayer extends Command {
    private final MusicPlayerSubsystem player;

    public PauseMusicPlayer(MusicPlayerSubsystem player) {
        this.player = player;
        addRequirements(player);
    }

    @Override
    public void initialize() {
        player.pauseMusic();
    }

    @Override
    public void end(boolean interrupted) {
        player.playLoadedTrack();
    }

}
