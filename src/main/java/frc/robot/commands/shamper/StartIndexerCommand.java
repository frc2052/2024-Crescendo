// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShamperSubsystem;

public class StartIndexerCommand extends Command {
  ShamperSubsystem shamper;
  DigitalInput input;

  public StartIndexerCommand(ShamperSubsystem shamper) {
    input = new DigitalInput(Constants.Shamper.INDEXER_SENSOR_ID);
    this.shamper = shamper;
    addRequirements(shamper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (input.get()) {
      shamper.stopIndexer();
    } else if (!shamper.indexerRunning()) {
      shamper.runIndexer();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shamper.stopIndexer();
  }
}
