package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shamper.ShamperShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
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
        addCommands(new ShamperShootCommand(shamper, indexer),
        new AutoDriveCommand(drivetrain, 1, -1, 0));
        
    }








}
