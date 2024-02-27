package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.GyroOffsetCommand;
import frc.robot.commands.indexer.IndexerIndexCommand;
import frc.robot.commands.shamper.ShamperAngleCommand;
import frc.robot.commands.shamper.ShamperManualShootCommand;
import frc.robot.commands.shamper.ShamperShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.calculator.ShootingAngleCalculator;


public class BasicAuto extends SequentialCommandGroup {
    private String pointOnPodium;
    private DrivetrainSubsystem drivetrain;
    private ShamperSubsystem shamper;
    private IndexerSubsystem indexer;
    

    public BasicAuto (DrivetrainSubsystem drivetrain, ShamperSubsystem shamper, IndexerSubsystem indexer) {
        this.pointOnPodium = pointOnPodium;
        this.drivetrain = drivetrain;
        this.shamper = shamper;
        this.indexer = indexer;
        addCommands(new ParallelDeadlineGroup(new WaitCommand(1.5), new ShamperAngleCommand(shamper, Constants.Shamper.Angle.SUB)),
        new ParallelDeadlineGroup(new WaitCommand(2.5), new ShamperManualShootCommand(shamper, ShamperSpeed.SPEAKER_IDLE)),
        new ParallelDeadlineGroup(new WaitCommand(1), new IndexerIndexCommand(indexer)),
        new AutoDriveCommand(drivetrain, 1.5, -0.25, 0));
        new GyroOffsetCommand(180);
        
    }








}
