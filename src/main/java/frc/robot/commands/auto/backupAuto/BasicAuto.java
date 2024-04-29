package frc.robot.commands.auto.backupAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.commands.drive.AutoDriveCommand;
import frc.robot.commands.indexer.IndexerIndexCommand;
import frc.robot.commands.shamper.pivot.ShamperAngleCommand;
import frc.robot.commands.shamper.shoot.ShamperManualShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;


public class BasicAuto extends SequentialCommandGroup {
    public BasicAuto (DrivetrainSubsystem drivetrain, ShamperSubsystem shamper, IndexerSubsystem indexer) {
        addCommands(new ParallelDeadlineGroup(new WaitCommand(1.5), new ShamperAngleCommand(shamper, Constants.Shamper.Angle.SUB)),
        new ParallelDeadlineGroup(new WaitCommand(2.5), new ShamperManualShootCommand(shamper, ShamperSpeed.SPEAKER_IDLE)),
        new ParallelDeadlineGroup(new WaitCommand(1), new IndexerIndexCommand(indexer)),
        new AutoDriveCommand(drivetrain, 1.5, -0.25, 0));
        
    }
}
