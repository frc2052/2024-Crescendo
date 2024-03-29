// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AimingCalculator;

public class DriveWhileAimingSpeakerSingleTag extends DriveCommand {
  private final ProfiledPIDController rotationController;
  /** Creates a new DriveWhileAimingSpeakerSingleTag. */
  public DriveWhileAimingSpeakerSingleTag(        
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier,
        BooleanSupplier fieldCentricSupplier,
        DrivetrainSubsystem drivetrain
    ) {
        super(xSupplier, ySupplier, () -> 0.0, fieldCentricSupplier, drivetrain);

        rotationController = new ProfiledPIDController(0.15, 0, 0, Constants.Drivetrain.AIM_PID_CONSTRAINT);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(0.5);

        addRequirements(drivetrain);
    }

  // Called when the command is initially scheduled.
  @Override
  protected double getRotation() {
      double goalAngleDegrees = AimingCalculator.calculateRobotAngleOneAprilTag();
      double deltaDegrees = RobotState.getInstance().getRotation2d360().getDegrees() - goalAngleDegrees;
      
      double rotation = rotationController.calculate(goalAngleDegrees, 0);
      Logger.recordOutput("Robot Angle", RobotState.getInstance().getRotation2d360().getDegrees());
      Logger.recordOutput("Speaker Tag PID Error", rotationController.getPositionError());
      if (AprilTagSubsystem.getInstance().isSeeingSpeakerTag()){
        return rotation;
      } else {
        return 0;
      }
  }
}
