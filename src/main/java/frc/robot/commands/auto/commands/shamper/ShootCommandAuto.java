// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commands.shamper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
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
    private double indexStartTime = 0;
    private ShootAngleConfig config;

    public ShootCommandAuto(ShamperSubsystem shamper, IndexerSubsystem indexer) {
        this.shamper = shamper;
        this.indexer = indexer;
        addRequirements(shamper, indexer);
  }

  @Override
  public void initialize() {
    indexStartTime = 0;
  }

  protected ShootAngleConfig getTargetAngle(){
    return ShootingAngleCalculator.getInstance().getShooterConfig(AimingCalculator.calculateDistanceToAimPoint(RobotState.getInstance().getRobotPose()));
  }

  @Override
  public void execute() {
    config = getTargetAngle();
    Logger.recordOutput("shoot speed calculated", config.getShooterSpeedVelocityRPS());
    Logger.recordOutput("angle calculated", config.getAngleDegrees());
    shamper.setShootSpeed(config.getShooterSpeedVelocityRPS(), config.getShooterSpeedVelocityRPS());
    shamper.setAngle(config.getAngleDegrees());

    if(shamper.shooterAtSpeed(config.getShooterSpeedVelocityRPS(), config.getShooterSpeedVelocityRPS()) && shamper.isAtGoalAngle()) {
        indexer.indexAll();
        if(indexStartTime == 0){
          indexStartTime = Timer.getFPGATimestamp();
        }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shamper.windDownShooter();
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return (!indexer.getNoteHeld() && !indexer.getNoteStaged());
  }
}
