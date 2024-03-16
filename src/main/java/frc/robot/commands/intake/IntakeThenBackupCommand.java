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

public class IntakeThenBackupCommand extends SequentialCommandGroup {
  /** Creates a new IntakeThenBackupCommand. */
  public IntakeThenBackupCommand(IntakeSubsystem intake, IndexerSubsystem indexer, ShamperSubsystem shamper) {
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
