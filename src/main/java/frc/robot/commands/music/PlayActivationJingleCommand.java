// package frc.robot.commands.music;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.MusicPlayerSubsystem;

// public class PlayActivationJingleCommand extends Command{
//     private final MusicPlayerSubsystem player;

//     public PlayActivationJingleCommand(MusicPlayerSubsystem player) {
//         this.player = player;
//         addRequirements(player);
//     }

//     @Override
//     public void initialize() {
//         player.stopMusic();
//         player.loadMusic("ActivationJingle.chrp");
//         player.playLoadedTrack();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         player.stopMusic();
//     }

// }
