// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper.shoot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.util.io.Dashboard;

public class ShamperCustomShotCommand extends Command {

  private Timer indexTimer;
  private boolean isFinished;

  private ShamperSubsystem shamper;
  private IndexerSubsystem indexer;
  /** Creates a new ShamperAmpCommand. */
  public ShamperCustomShotCommand(ShamperSubsystem shamper, IndexerSubsystem indexer) {
    this.shamper = shamper;
    this.indexer = indexer;
    indexTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shamper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  public double getValue(double value){
    double num = value;
    if(num > 1){
      num = 1;
    } else if (num < -1){
      num = -1;
    }

    return num;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lowerMotorSpeedPct = Dashboard.getInstance().getManualShotLower();
    double upperMotorSpeedPct = Dashboard.getInstance().getManualShotUpper();
    shamper.setShootSpeed(getValue(lowerMotorSpeedPct) * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, getValue(upperMotorSpeedPct) * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS);
    if(shamper.shooterAtSpeed(getValue(lowerMotorSpeedPct) * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, getValue(upperMotorSpeedPct) * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS)){
      indexTimer.restart();
      indexer.indexAll();
      if(indexTimer.get() > 0.25) {
        isFinished = true;
      }
    } else {
      isFinished = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shamper.stopShooter();
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
