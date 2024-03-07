// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.indexer.IndexerBackupCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeThenBackupCommand extends SequentialCommandGroup {
  private IndexerSubsystem indexer;
  private IntakeSubsystem intake;
  private ShamperSubsystem shamper;
  /** Creates a new IntakeThenBackupCommand. */
  public IntakeThenBackupCommand(IntakeSubsystem intake, IndexerSubsystem indexer, ShamperSubsystem shamper) {
    this.indexer = indexer;
    this.intake = intake;
    this.shamper = shamper;
    

    addCommands(
      new IntakeCommand(intake, indexer, shamper),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new IndexerBackupCommand(indexer),
        new IntakeReverseCommand(intake)
      )
    );
  }
}
