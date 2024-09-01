// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.shamper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.calculator.ShootAngleConfig;
import frc.robot.util.calculator.ShootingAngleCalculator;

public class ShootCommandAuto extends Command {
    private final ShamperSubsystem shamper;
    private final IndexerSubsystem indexer;
    private ShootAngleConfig config;

    public ShootCommandAuto(ShamperSubsystem shamper, IndexerSubsystem indexer) {
        this.shamper = shamper;
        this.indexer = indexer;
        addRequirements(shamper, indexer);
  }

  @Override
  public void initialize() {
  }

  protected ShootAngleConfig getTargetAngle(){
    return ShootingAngleCalculator.getInstance().getShooterConfig(AimingCalculator.calculateDistanceToSpeaker(RobotState.getInstance().getRobotPose()));
  }

  @Override
  public void execute() {
    config = getTargetAngle();
    Logger.recordOutput("angle calculated", config.getAngleDegrees());
    shamper.setShootSpeed(config.getLowerShooterSpeedVelocityRPS(), config.getUpperShooterSpeedVelocityRPS());
    shamper.setAngle(config.getAngleDegrees());

    if(shamper.shooterAtSpeed(config.getLowerShooterSpeedVelocityRPS(), config.getUpperShooterSpeedVelocityRPS()) && shamper.isAtGoalAngle()) {
        indexer.indexAll();
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return !(RobotState.getInstance().getNoteHeldDetected() || RobotState.getInstance().getNoteStagedDetected());
  }
}
