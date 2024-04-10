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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.BooleanSupplier;

public class FeedWhileMovingCommand extends DriveCommand {
  private final PIDController rotationController;
  private final ShamperSubsystem shamper;
  private final IndexerSubsystem indexer;
  private final double blueIndexLine;
  private final double redIndexLine;
  private double robotSpeed;
  /** Creates a new AimToNoteLobPointCommand. */
  public FeedWhileMovingCommand(        
  DoubleSupplier xSupplier, 
  DoubleSupplier ySupplier,
  BooleanSupplier fieldCentricSupplier,
  DrivetrainSubsystem drivetrain,
  ShamperSubsystem shamper,
  IndexerSubsystem indexer) {
    super(xSupplier, ySupplier, () -> 0.0, fieldCentricSupplier, drivetrain);

    rotationController = new PIDController(2.5, 0, 0.1);
    rotationController.enableContinuousInput(0, 360);
    rotationController.setTolerance(2);

    this.shamper = shamper;
    this.indexer = indexer;

    blueIndexLine = Constants.FieldAndRobot.BLUE_LOB_LINE;
    redIndexLine = Constants.FieldAndRobot.RED_LOB_LINE;

    robotSpeed = 0;

    shamper.setShootSpeed(ShamperSpeed.LOB);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shamper);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Math.sqrt(
    Math.pow(RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond, 2) +
    Math.pow(RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond, 2)
    );

    robotSpeed = speed;

    double angle = 45 + (speed * Constants.FieldAndRobot.FEED_WHILE_MOVING_ANGLE_MULTIPLYER);

    shamper.setAngle(angle);

    double poseX = RobotState.getInstance().getRobotPose().getX();

    if (RobotState.getInstance().isRedAlliance()){
      if (poseX > redIndexLine && shamper.isAtGoalAngle() && shamper.shooterAtSpeed(ShamperSpeed.LOB.getLower(), ShamperSpeed.LOB.getUpper())) {
        indexer.indexAll();
      }
    } else {
      if (poseX < blueIndexLine && shamper.isAtGoalAngle() && shamper.shooterAtSpeed(ShamperSpeed.LOB.getLower(), ShamperSpeed.LOB.getUpper())) {
        indexer.indexAll();
      }
    }
  }

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

    goalPoint.plus(new Translation2d(
      RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond * Constants.FieldAndRobot.FEED_WHILE_MOVING_VELOCITY_MULTIPLYER,
      RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond * Constants.FieldAndRobot.FEED_WHILE_MOVING_VELOCITY_MULTIPLYER
    ));

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
