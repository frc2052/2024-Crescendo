// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AimingCalculator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.BooleanSupplier;

public class AimToNoteLobPointCommand extends DriveCommand {
  private final PIDController rotationController;
  /** Creates a new AimToNoteLobPointCommand. */
  public AimToNoteLobPointCommand(        
  DoubleSupplier xSupplier, 
  DoubleSupplier ySupplier,
  BooleanSupplier fieldCentricSupplier,
  DrivetrainSubsystem drivetrain) {
    super(xSupplier, ySupplier, () -> 0.0, fieldCentricSupplier, drivetrain);

    rotationController = new PIDController(2.5, 0, 0.1);
    rotationController.enableContinuousInput(0, 360);
    rotationController.setTolerance(2);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  protected double getRotation() {
    Translation2d goalPoint;
    if (!RobotState.getInstance().isRedAlliance()) {
      goalPoint = Constants.FieldAndRobot.BLUE_LOB_POINT;
    } else {
      goalPoint = Constants.FieldAndRobot.RED_LOB_POINT;
    }
    double rotation = AimingCalculator.angleToPoint(goalPoint, RobotState.getInstance().getRobotPose(), RobotState.getInstance().getChassisSpeeds());

    if(Math.abs(rotation) < 0.04){
      // needs at least 4% power to make robot turn
      rotation = Math.copySign(0.04, rotation);
  } else if (Math.abs(rotation) > 0.4) {
      rotation = Math.copySign(0.4, rotation);
  } 

  return rotation;
  }
  
}
