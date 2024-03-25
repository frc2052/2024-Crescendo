// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.ctre.phoenix6.ISerializable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.shamper.lookup.ShamperAutoAngleCommand;
import frc.robot.commands.shamper.lookup.ShamperAutoShootCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.calculator.ShootAngleConfig;
import frc.robot.util.calculator.ShootingAngleCalculator;

public class MegaAutoCommand extends Command {
  protected final ShamperSubsystem shamper;
  private final IndexerSubsystem indexer;
  private final IntakeSubsystem intake;
  private RobotState robotState;
  /** Creates a new MegaAutoCommand. */
  public MegaAutoCommand(ShamperSubsystem shamper, IndexerSubsystem indexer, IntakeSubsystem intake) {
    this.shamper = shamper;
    this.indexer = indexer;
    this.intake = intake;

    robotState = RobotState.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {}

  private boolean isReady = false;

  @Override
  public void execute() {
    ShootAngleConfig config = getShootConfig();
    shamper.setAngle(config.getAngleDegrees());
    shamper.setShootSpeed(config.getShooterSpeedVelocityRPS(), config.getShooterSpeedVelocityRPS());
    //shamper.setShootSpeed(.9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, .9 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS);

    // isReady = true; // isReady || shamperReady();

    // if (isReady) {
    //   indexer.indexAll();
    //   intake.intake();
    // }


    if (indexer.getNoteStaged()) { 
      if(shamperReady() && robotState.distanceToSpeaker() < 4) {
        indexer.indexAll();
      } else {
        indexer.stop();
      }
    } else {
      intake.intake();
      if(indexer.getNoteHeld()){
          indexer.loadSlow();
      } else {
          indexer.load();
      }
    }
  }

  public ShootAngleConfig getShootConfig(){
    ShootAngleConfig config = ShootingAngleCalculator.getInstance().getShooterConfig(AimingCalculator.calculateDistanceToSpeaker(RobotState.getInstance().getRobotPose()));
    return config;

  }

  public boolean shamperReady(){
    return shamper.shooterAtSpeed(getShootConfig().getShooterSpeedVelocityRPS(), getShootConfig().getShooterSpeedVelocityRPS());// && shamper.isAtGoalAngle(4);
  }

  @Override
  public void end(boolean interrupted) {
    shamper.stopShooter();
    intake.stop();
    indexer.stop();
    System.out.println("WOMP WOMP");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}