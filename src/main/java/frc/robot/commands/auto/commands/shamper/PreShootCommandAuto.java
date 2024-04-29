// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.calculator.ShootAngleConfig;
import frc.robot.util.calculator.ShootingAngleCalculator;

public class PreShootCommandAuto extends Command {
  private ShamperSubsystem shamper;
  /** Creates a new PreShootCommandAuto. */
  public PreShootCommandAuto(ShamperSubsystem shamper) {
    this.shamper = shamper;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShootAngleConfig config = ShootingAngleCalculator.getInstance().getShooterConfig(AimingCalculator.calculateDistanceToSpeaker(RobotState.getInstance().getRobotPose()));
    shamper.setAngle(config.getAngleDegrees());
    shamper.setShootSpeed(config.getLowerShooterSpeedVelocityRPS(), config.getUpperShooterSpeedVelocityRPS());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
