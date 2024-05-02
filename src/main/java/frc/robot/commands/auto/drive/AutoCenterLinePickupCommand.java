// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCenterLinePickupCommand extends SequentialCommandGroup {
  private RobotState robotState;
  private boolean secondAttempt;
  private boolean yay;

   
  private DrivetrainSubsystem drivetrain;
  private ForwardPixySubsystem pixy;
  private IntakeSubsystem intake;
  private IndexerSubsystem indexer;
  private ShamperSubsystem shamper;
  /** Creates a new AutoCenterLinePickupCommand. */
  public AutoCenterLinePickupCommand(
      DrivetrainSubsystem drivetrain,
      ForwardPixySubsystem pixy,
      IntakeSubsystem intake,
      IndexerSubsystem indexer,
      ShamperSubsystem shamper
      
    ) {
      robotState = RobotState.getInstance();
      this.drivetrain = drivetrain;
      this.pixy = pixy;
      this.intake = intake;
      this.indexer = indexer;
      this. shamper = shamper;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

      addCommands(
        // first attempt
        new ParallelRaceGroup(
          // burger midline
          // new PastCenterlineCommand(),
          new IntakeCommand(intake, indexer, shamper),
          new AutoDriveWhileGamePieceAlign(0.5, 1, drivetrain, pixy),
          checkSecondAttempt()
        ),

        new ConditionalCommand(
          new ParallelCommandGroup(
            // we failed first note so rotate down the line
            new AutoAimDownCenterlineCommand(),

            // second attempt (facing down the line)
            new ParallelRaceGroup(
              // hotdog midline
              // new PastMidlineCommand(),
              new IntakeCommand(intake, indexer, shamper),
              new AutoDriveWhileGamePieceAlign(0.5, 1, drivetrain, pixy)
            )
          ), 
          new InstantCommand(() -> yay = true), 
          () -> secondAttempt
        )
      );
    }

    public InstantCommand checkSecondAttempt() {
      if(robotState.getNoteHeldDetected() || robotState.getNoteStagedDetected()) {
        return new InstantCommand(() -> this.secondAttempt = false);

      } else {
        return new InstantCommand(() -> this.secondAttempt = true);
      }
    }
}
