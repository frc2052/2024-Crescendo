// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.calculator.ShootAngleConfig;
import frc.robot.util.calculator.ShootingAngleCalculator;

public class MegaAutoBackupCommand extends MegaAutoCommand {
  /** Creates a new MegaAutoBackupCommand. */
  public MegaAutoBackupCommand(ShamperSubsystem shamper, IndexerSubsystem indexer, IntakeSubsystem intake) {
    super(shamper, indexer, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
    public ShootAngleConfig getShootConfig(){
    Pose2d robotPose = RobotState.getInstance().getRobotPose();
    System.out.println(robotPose.getY());
//    if(!RobotState.getInstance().isRedAlliance()){
//      robotPose = robotPose.transformBy(new Transform2d(new Translation2d(-1.5,0), new Rotation2d(0)));
//    }else{
      robotPose = robotPose.plus(new Transform2d(new Translation2d(0.4,0), new Rotation2d(0)));
      //System.out.println(robotPose.getX());
      // robotPose = robotPose.transformBy(new Transform2d(new Translation2d(-1,0), new Rotation2d(0)));
//    }
    ShootAngleConfig config = ShootingAngleCalculator.getInstance().getShooterConfig(AimingCalculator.calculateDistanceToSpeaker(robotPose));
    config = new ShootAngleConfig(config.getDistanceCentimeters(), config.getAngleDegrees(), .9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS);
    System.out.println(robotPose.getY());
    return config;
  }

  @Override
  public boolean shamperReady(){
    return shamper.shooterAtSpeed(getShootConfig().getShooterSpeedVelocityRPS(), getShootConfig().getShooterSpeedVelocityRPS(), 10) && shamper.isAtGoalAngle();
  }
}
