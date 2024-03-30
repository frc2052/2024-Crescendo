// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper.shoot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;

public class ShamperLobCommand extends Command {
  private ShamperSubsystem shamper;
  // private DrivetrainSubsystem drivetrain;
  private IndexerSubsystem indexer;
  // private final PIDController rotationController;
  /** Creates a new ShamperLobCommand. */
  public ShamperLobCommand(ShamperSubsystem shamper, IndexerSubsystem indexer) {
    this.shamper = shamper;
    this.indexer = indexer;
    // this.drivetrain = drivetrain;

    // rotationController = new PIDController(0.4, 0, 0.1);
    // rotationController.enableContinuousInput(0, 360);
    // rotationController.setTolerance(2);


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shamper, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shamper.setShootSpeed(ShamperSpeed.LOB);
    shamper.setAngle(Constants.Shamper.Angle.LOB);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double rotation = rotationController.calculate(Constants.Drivetrain.LOB_ANGLE, RobotState.getInstance().getRobotPose().getRotation().getDegrees());

    // if(Math.abs(rotation) < 0.04){
    //   // needs at least 4% power to make robot turn
    //   rotation = Math.copySign(0.04, rotation);
    // } else if (Math.abs(rotation) > 0.4) {
    //   rotation = Math.copySign(0.4, rotation);
    // } 

    // drivetrain.drive(0, 0, rotation, true);

    if(shamper.isAtGoalAngle() && shamper.shooterAtSpeed(ShamperSpeed.LOB.getLower(), ShamperSpeed.LOB.getUpper())){
      indexer.indexAll();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shamper.stopShooter();
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
