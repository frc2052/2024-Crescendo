// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerBackupCommand extends Command {
  private IndexerSubsystem indexer;
  private Timer runBackTimer;
  private boolean isFinished;
  /** Creates a new IndexerIndex. */
  public IndexerBackupCommand(IndexerSubsystem indexer) {
    this.indexer = indexer;
    runBackTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runBackTimer.reset();
    runBackTimer.start();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(runBackTimer.get());
    if(runBackTimer.get() < 0.1){
      indexer.slideBack();
    } else {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runBackTimer.reset();
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
