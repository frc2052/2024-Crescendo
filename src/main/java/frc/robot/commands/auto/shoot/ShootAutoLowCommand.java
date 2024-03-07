// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.shoot;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.util.calculator.ShootAngleConfig;

public class ShootAutoLowCommand extends ShootCommandAuto {
  /** Creates a new ShootAutoLowCommand. */
  public ShootAutoLowCommand(ShamperSubsystem shamper, IndexerSubsystem indexer) {
    super(shamper, indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  protected ShootAngleConfig getTargetAngle(){
    ShootAngleConfig config = super.getTargetAngle();
    return new ShootAngleConfig(config.getDistanceCentimeters(), config.getAngleDegrees() - 3, config.getShooterSpeedVelocityRPS());
  }
}
