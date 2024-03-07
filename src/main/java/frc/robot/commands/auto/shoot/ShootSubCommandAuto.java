// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.shoot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Shamper;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.AimingCalculator;
import frc.robot.util.calculator.ShootAngleConfig;
import frc.robot.util.calculator.ShootingAngleCalculator;

public class ShootSubCommandAuto extends Command {
    private final ShamperSubsystem shamper;
    private final IndexerSubsystem indexer;
    private double indexStartTime = 0;

    public ShootSubCommandAuto(ShamperSubsystem shamper, IndexerSubsystem indexer) {
        this.shamper = shamper;
        this.indexer = indexer;
        addRequirements(shamper, indexer);
  }

  @Override
  public void initialize() {
    indexStartTime = 0;
  }

  @Override
  public void execute() {
    shamper.setShootSpeed(ShamperSpeed.SUB);
    shamper.setAngle(Constants.Shamper.Angle.SUB);

    if(shamper.shooterAtSpeed(ShamperSpeed.SUB.getLower(),ShamperSpeed.SUB.getUpper()) && shamper.isAtGoalAngle()) {
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
    return (indexStartTime > 0 && Timer.getFPGATimestamp() - indexStartTime > 0.25);
  }
}
