// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class StartIndexerCommand extends Command {
  ShooterSubsystem shooter;
  DigitalInput input;

  public StartIndexerCommand(ShooterSubsystem shooter) {
    input = new DigitalInput(Constants.VerticalShooter.INDEXER_SENSOR_ID);
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (input.get()) {
      shooter.stopIndexer();
    } else if (!shooter.indexerRunning()) {
      shooter.runIndexer();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopIndexer();
  }
}
