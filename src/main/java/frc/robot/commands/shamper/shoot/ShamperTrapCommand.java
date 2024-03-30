// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;

public class ShamperTrapCommand extends Command {
  private ShamperSubsystem shamper;
  private IndexerSubsystem indexer;
  private Timer indexTimer;
  private boolean isFinished;
  /** Creates a new ShamperTrapCommand. */
  public ShamperTrapCommand(ShamperSubsystem shamper, IndexerSubsystem indexer) {
    this.shamper = shamper;
    this.indexer = indexer;
    indexTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shamper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shamper.setShootSpeedPct(0.25, 0.03);
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("executing");
    if(shamper.shooterAtSpeed(0.25 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS, 0.03 * Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS)){
      System.out.println("starting timer");
      indexTimer.restart();
      indexer.indexAll();
      if(indexTimer.get() > 0.25) {
        System.out.println("done index");
        isFinished = true;
      }
    } else {
      isFinished = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexTimer.reset();
    indexer.stop();
    shamper.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
