// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShamperSubsystem extends SubsystemBase {

  private final static TalonFX lowerMotor = new TalonFX(Constants.Shamper.Motors.LOWER_MOTOR_ID);
  private final static TalonFX upperMotor = new TalonFX(Constants.Shamper.Motors.UPPER_MOTOR_ID);
  private final static TalonFX rotationMotor = new TalonFX(Constants.Shamper.Motors.PIVOT_MOTOR_ID);
  private final static TalonFX indexMotor = new TalonFX(Constants.Shamper.INDEX_MOTOR_ID);
  private final AnalogEncoder rotationEncoder;

  private ProfiledPIDController lowerMotorController;
  private ProfiledPIDController upperMotorController;
  private ProfiledPIDController rotationMotorController;

  public ShamperSubsystem() {

    lowerMotorController = new ProfiledPIDController(
    Constants.Shamper.LOWER_MOTOR_KP,
    Constants.Shamper.LOWER_MOTOR_KI,
    Constants.Shamper.LOWER_MOTOR_KD, 
    new TrapezoidProfile.Constraints(
    Constants.Shamper.LOWER_MOTOR_MAX_VELOCITY,
    Constants.Shamper.LOWER_MOTOR_MAX_ACCELERATION));

    upperMotorController = new ProfiledPIDController(
    Constants.Shamper.UPPER_MOTOR_KP,
    Constants.Shamper.UPPER_MOTOR_KI,
    Constants.Shamper.UPPER_MOTOR_KD,      
    new TrapezoidProfile.Constraints(
    Constants.Shamper.UPPER_MOTOR_MAX_VELOCITY,
    Constants.Shamper.UPPER_MOTOR_MAX_ACCELERATION));
    
    rotationMotorController = new ProfiledPIDController(
    Constants.Shamper.PIVOT_MOTOR_KP,
    Constants.Shamper.PIVOT_MOTOR_KI,
    Constants.Shamper.PIVOT_MOTOR_KD,      
    new TrapezoidProfile.Constraints(
    Constants.Shamper.PIVOT_MOTOR_MAX_VELOCITY,
    Constants.Shamper.PIVOT_MOTOR_MAX_ACCELERATION));

    rotationEncoder = new AnalogEncoder(Constants.Shamper.Motors.PIVOT_ENCODER_ID);
    rotationEncoder.reset();

    lowerMotor.setInverted(Constants.Shamper.LOWER_MOTOR_IS_INVERTED);
    upperMotor.setInverted(Constants.Shamper.UPPER_MOTOR_IS_INVERTED);
    rotationMotor.setInverted(Constants.Shamper.ROTATION_MOTOR_IS_INVERTED);
  }

  public void stop() {
    lowerMotor.set(TalonFXControlMode.PercentOutput, ShamperSpeeds.OFF.getLowerPCT());
    upperMotor.set(TalonFXControlMode.PercentOutput, ShamperSpeeds.OFF.getUpperPCT());
  }

  public void setSpeed(ShamperSpeeds speeds) {
    lowerMotor.set(TalonFXControlMode.PercentOutput, lowerMotorController.calculate(getLowerShamperSpeed(), speeds.getLowerPCT()));
    upperMotor.set(TalonFXControlMode.PercentOutput, upperMotorController.calculate(getUpperShamperSpeed(), speeds.getUpperPCT()));
  }

  public void setUpperShamperSpeed(double upperShamperMotorSpeedPCT) {
    upperMotor.set(TalonFXControlMode.PercentOutput, upperMotorController.calculate(getUpperShamperSpeed(), upperShamperMotorSpeedPCT));
  }

  public void setLowerShamperSpeed(double lowerShamperMotorSpeedPCT) {
    lowerMotor.set(TalonFXControlMode.PercentOutput, lowerMotorController.calculate(getLowerShamperSpeed(), lowerShamperMotorSpeedPCT));
  }

  public void setAngle(double shooterGoalAngle) {
    // for safety. don't want the robot breaking
    if (shooterGoalAngle < Constants.Shamper.Angle.MINIMUM) {
      shooterGoalAngle = Constants.Shamper.Angle.MINIMUM;
    } else if (shooterGoalAngle > Constants.Shamper.Angle.MAXIMUM) {
      shooterGoalAngle = Constants.Shamper.Angle.MAXIMUM;
    }
    
    rotationMotor.set(TalonFXControlMode.Position, rotationMotorController.calculate(
    ((getShamperAngle() / 360) * Constants.MotorConstants.FALCON500_TICKS_PER_ROTATION), 
    ((shooterGoalAngle / 360) * Constants.MotorConstants.FALCON500_TICKS_PER_ROTATION) * Constants.Shamper.PIVOT_GEAR_RATIO));

  }

  public double getShamperAngle() {
    return rotationEncoder.get() * 360;
  }

  public boolean checkShamperAngleValidity(double testShamperAngle) {
      if (testShamperAngle < Constants.Shamper.Angle.MINIMUM) {
      return false;
    } else if (testShamperAngle > Constants.Shamper.Angle.MAXIMUM) {
      return false;
    }
    return true;
  }

  public double getUpperShamperSpeed() {
    return upperMotor.getSelectedSensorVelocity();
  }

  public double getLowerShamperSpeed() {
    return lowerMotor.getSelectedSensorVelocity();
  }  

  public void runIndexer() {
    indexMotor.set(TalonFXControlMode.Velocity, Constants.Shamper.INDEX_SPEED_TPS);
  }

  public void stopIndexer() {
    indexMotor.set(TalonFXControlMode.Velocity, 0);
  }

  public boolean indexerRunning() {
    return indexMotor.getSelectedSensorVelocity() != 0.01;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public static enum ShamperSpeeds {
    OFF(0, 0),
    SPEAKER_IDLE(Constants.Shamper.Speed.LOWER_SHAMPER_SPEAKER_IDLE_SPEED_PCT, Constants.Shamper.Speed.UPPER_SHAMPER_SPEAKER_IDLE_SPEED_PCT),
    AMP_IDLE(Constants.Shamper.Speed.LOWER_SHAMPER_AMP_IDLE_SPEED_PCT, Constants.Shamper.Speed.UPPER_SHAMPER_AMP_IDLE_SPEED_PCT),
    SPEAKER(Constants.Shamper.Speed.LOWER_SHAMPER_SPEAKER_SPEED_PCT, Constants.Shamper.Speed.UPPER_SHAMPER_SPEAKER_SPEED_PCT),
    AMP(Constants.Shamper.Speed.LOWER_SHAMPER_AMP_SPEED_PCT, Constants.Shamper.Speed.UPPER_SHAMPER_AMP_SPEED_PCT);

    private final int lowerPercentOutput;
    private final int upperPercentOutput;

    private ShamperSpeeds(int lowerPercentOutput, int upperPercentOutput) {
      this.lowerPercentOutput = lowerPercentOutput;
      this.upperPercentOutput = upperPercentOutput;
    }

    public int getLowerPCT() {
      return lowerPercentOutput;
    }

    public int getUpperPCT() {
      return upperPercentOutput;
    }


  }
     public static TalonFX getUpperTalonFX() {
      return upperMotor;
    }

    public static TalonFX getLowerTalonFX() {
      return lowerMotor;
    }

    public static TalonFX getRotationTalonFX() {
      return rotationMotor;
    } 
}