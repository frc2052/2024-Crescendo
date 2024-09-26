// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveWhileAimToAngle;
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
  private boolean startRightSide;
  private Rotation2d rotationDownLine;
   
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
      ShamperSubsystem shamper,
      Rotation2d rotationDirection
    ) {
      robotState = RobotState.getInstance();
      this.drivetrain = drivetrain;
      this.pixy = pixy;
      this.intake = intake;
      this.indexer = indexer;
      this. shamper = shamper;

      // check if coming from the left or right of field
      this.rotationDownLine = rotationDirection;

      addCommands(
        // first attempt
        new ParallelRaceGroup(
          // burger midline
          new AutoPastHamburgerCenterlineCommand(),
          new IntakeCommand(intake, indexer, shamper),
          new AutoDriveWhileGamePieceAlign(0.5, 1, 0.5, drivetrain, pixy)
        ),
        
        checkSecondAttempt(),

        new ConditionalCommand(
          new SequentialCommandGroup(
            // we failed first note so rotate down the line
            new DriveWhileAimToAngle(() -> 0, () -> 0, () -> rotationDownLine, () -> false, drivetrain),

            // second attempt (facing down the line)
            new ParallelRaceGroup(
              // hotdog midline
              // new AutoPastHotdogCenterlineCommand(),
              new IntakeCommand(intake, indexer, shamper),
              new AutoDriveWhileGamePieceAlign(0.5, 1, 0.5, drivetrain, pixy)
            )
          ), 
          // something has to happen on false?
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
