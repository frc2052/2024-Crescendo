// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;

public class ShamperManualShootCommand extends Command {
  private ShamperSubsystem shamper;
  private IndexerSubsystem indexer;
  private double goalSpeed;
  /** Creates a new ShamperManualShootCommand. */
    public ShamperManualShootCommand(ShamperSubsystem shamper, IndexerSubsystem indexer, double goalSpeed) {
        this.shamper = shamper;
        this.goalSpeed = goalSpeed;
        this.indexer = indexer;

        addRequirements(shamper, indexer);
    }

    @Override
    public void initialize() {
        shamper.setShootSpeed(goalSpeed, goalSpeed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        shamper.setShootSpeed(ShamperSpeed.OFF);
    }
}
