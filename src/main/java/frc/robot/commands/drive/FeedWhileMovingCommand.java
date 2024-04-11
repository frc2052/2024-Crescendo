// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.AimingCalculator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.BooleanSupplier;

public class FeedWhileMovingCommand extends DriveCommand {
  private final PIDController rotationController;
  private SimpleMotorFeedforward rotationFeedForward;
  private boolean isOnTarget;
  /** Creates a new AimToNoteLobPointCommand. */
  public FeedWhileMovingCommand(        
  DoubleSupplier xSupplier, 
  DoubleSupplier ySupplier,
  BooleanSupplier fieldCentricSupplier,
  DrivetrainSubsystem drivetrain) {
    super(xSupplier, ySupplier, () -> 0.0, fieldCentricSupplier, drivetrain);

        rotationController = new PIDController(0.3, 0, 0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(0.087, 0.087);

        rotationFeedForward = new SimpleMotorFeedforward(0.013, 0.013, 0);

        isOnTarget = false;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  protected double getRotation() {
    Translation2d goalPoint;
    double multiplier = Constants.FieldAndRobot.FEED_WHILE_MOVING_VELOCITY_MULTIPLIER;
    if (!RobotState.getInstance().isRedAlliance()) {
      goalPoint = Constants.FieldAndRobot.BLUE_LOB_POINT;
      multiplier = multiplier * -1;
    } else {
      goalPoint = Constants.FieldAndRobot.RED_LOB_POINT;
    }
    double robotX = RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond;
    double robotY = RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond;

    goalPoint = goalPoint.plus(new Translation2d(
      robotX * multiplier,
      robotY * multiplier
    ));

    Logger.recordOutput("Lob Goal Point", goalPoint);

    double currentAngle = RobotState.getInstance().getRobotPose().getRotation().getRadians();
    double goalAngle = AimingCalculator.angleToPoint(goalPoint, RobotState.getInstance().getRobotPose(), RobotState.getInstance().getChassisSpeeds()) + Math.toRadians(Constants.Drivetrain.AIM_OFFSET_DEGREES);

    double rotation = rotationController.calculate(currentAngle, goalAngle);

    rotation = rotation + rotationFeedForward.calculate(rotation);

    isOnTarget = Math.abs(currentAngle - goalAngle) < Math.toRadians(Constants.Drivetrain.AIM_TOLERANCE_DEGREES);

    if(Math.abs(rotation) < 0.04){
      // needs at least 4% power to make robot turn
      rotation = Math.copySign(0.04, rotation);
    } else if (Math.abs(rotation) > 0.4) {
        rotation = Math.copySign(0.4, rotation);
    } 

    return rotation;
  }
}
