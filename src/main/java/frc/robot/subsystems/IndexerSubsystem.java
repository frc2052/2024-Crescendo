// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkFlex lowerMotor;
  private final CANSparkMax upperMotor;
  private final DigitalInput noteDetector;

  public IndexerSubsystem() {
    lowerMotor = new CANSparkFlex(Constants.CAN.LOWER_INDEX_MOTOR_ID, MotorType.kBrushless);
    upperMotor = new CANSparkMax(Constants.CAN.SHAMPER_INDEX_ID, MotorType.kBrushless);
    noteDetector = new DigitalInput(Constants.Indexer.INDEXER_SENSOR_PIN);

    upperMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lowerMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
  }

  public void indexLower() {
    lowerMotor.set(Constants.Indexer.LOWER_INDEX_SPEED_PCT);
  }

  public void indexUpper() {
    upperMotor.set(Constants.Indexer.UPPER_INDEX_SPEED_PCT);
  }

  public void indexAll() {
    upperMotor.set(Constants.Indexer.UPPER_INDEX_SPEED_PCT);
    lowerMotor.set(Constants.Indexer.LOWER_INDEX_SPEED_PCT);
  }

  public void reverse() {
    upperMotor.set(-Constants.Indexer.UPPER_INDEX_SPEED_PCT);
    lowerMotor.set(-Constants.Indexer.LOWER_INDEX_SPEED_PCT);
  }

  public void stop() {
    upperMotor.stopMotor();
    lowerMotor.stopMotor();
  }

  public double getUpperIndexerSpeed() {
    return upperMotor.get();
  }

  public double getLowerIndexerSpeed() {
    return lowerMotor.get();
  }

  public boolean getNoteDetector() {
    return noteDetector.get();
  }

  @Override
  public void periodic() {
    boolean noteDetected = noteDetector.get();
    RobotState.getInstance().updateNoteDetected(noteDetected);
  }
}
