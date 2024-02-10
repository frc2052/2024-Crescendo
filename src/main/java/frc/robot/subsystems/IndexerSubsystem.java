// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkMax upperMotor;
  private final CANSparkFlex lowerMotor;

  public IndexerSubsystem() {
    upperMotor = new CANSparkMax(Constants.Indexer.LOWER_MOTOR_ID, MotorType.kBrushless);
    lowerMotor = new CANSparkFlex(Constants.Indexer.UPPER_MOTOR_ID, MotorType.kBrushless);

    upperMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lowerMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
  }

  public void runMotors() {
    runUpperMotor();
    runLowerMotor();
  }

  public void stopMotors() {
    stopUpperMotor();
    stopLowerMotor();
  }

  public void runUpperMotor() {
    upperMotor.set(Constants.Indexer.UPPER_SPEED_PCT);
  }

  public void runLowerMotor() {
    lowerMotor.set(Constants.Indexer.LOWER_SPEED_PCT);
  }

  public void stopUpperMotor() {
    upperMotor.stopMotor();
  }

  public void stopLowerMotor() {
    lowerMotor.stopMotor();
  }

  public double getUpperIndexerSpeed() {
    return upperMotor.get();
  }

  public double getLowerIndexerSpeed() {
    return lowerMotor.get();
  }

  @Override
  public void periodic() {
  }
}
