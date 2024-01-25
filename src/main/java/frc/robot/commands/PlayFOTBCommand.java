package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MusicPlayerSubsystem;

public class PlayFOTBCommand extends Command{
    private final MusicPlayerSubsystem player;

    public PlayFOTBCommand(MusicPlayerSubsystem player) {
        this.player = player;
        addRequirements(player);
    }

    @Override
    public void initialize() {
        player.stopMusic();
        player.loadMusic("FlightOfTheBumblebees.chrp");
        player.playLoadedTrack();
    }

    @Override
    public void end(boolean interrupted) {
        player.stopMusic();
    }

}
