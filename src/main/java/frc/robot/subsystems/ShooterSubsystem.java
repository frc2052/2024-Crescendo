// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX lowerMotor;
  private final TalonFX upperMotor;
  private final TalonFX rotationMotor;
  private final TalonFX indexMotor;
  private final AnalogEncoder rotationEncoder;

  private ProfiledPIDController lowerMotorController;
  private ProfiledPIDController upperMotorController;
  private ProfiledPIDController rotationMotorController;

  public ShooterSubsystem() {
    lowerMotor = new TalonFX(Constants.Shamper.UPPER_SHOOTER_MOTOR_ID);
    lowerMotorController = new ProfiledPIDController(
    Constants.Shamper.LOWER_SHOOTER_KP,
    Constants.Shamper.LOWER_SHOOTER_KI,
    Constants.Shamper.LOWER_SHOOTER_KD, 
    new TrapezoidProfile.Constraints(
    Constants.Shamper.LOWER_SHOOTER_MAX_VELOCITY,
    Constants.Shamper.LOWER_SHOOTER_MAX_ACCELORATION));

    upperMotor = new TalonFX(Constants.Shamper.LOWER_SHOOTER_MOTOR_ID);
    upperMotorController = new ProfiledPIDController(
    Constants.Shamper.UPPER_SHOOTER_KP,
    Constants.Shamper.UPPER_SHOOTER_KI,
    Constants.Shamper.UPPER_SHOOTER_KD,      
    new TrapezoidProfile.Constraints(
    Constants.Shamper.UPPER_SHOOTER_MAX_VELOCITY,
    Constants.Shamper.UPPER_SHOOTER_MAX_ACCELORATION));

    rotationMotor = new TalonFX(Constants.Shamper.ROTATION_SHOOTER_MOTOR_ID);
    rotationMotorController = new ProfiledPIDController(
    Constants.Shamper.ROTATION_SHOOTER_KP,
    Constants.Shamper.ROTATION_SHOOTER_KI,
    Constants.Shamper.ROTATION_SHOOTER_KD,      
    new TrapezoidProfile.Constraints(
    Constants.Shamper.ROTATION_SHOOTER_MAX_VELOCITY,
    Constants.Shamper.ROTATION_SHOOTER_MAX_ACCELORATION));

    indexMotor = new TalonFX(Constants.Shamper.INDEX_MOTOR_ID);

    rotationEncoder = new AnalogEncoder(Constants.Shamper.ROTATION_ENCODER_ID);
    rotationEncoder.reset();

    lowerMotor.setInverted(Constants.Shamper.LOWER_MOTOR_IS_INVERTED);
    upperMotor.setInverted(Constants.Shamper.UPPER_MOTOR_IS_INVERTED);
    rotationMotor.setInverted(Constants.Shamper.ROTATION_MOTOR_IS_INVERTED);
  }

  public void stop() {
    lowerMotor.set(TalonFXControlMode.Velocity, ShooterSpeeds.OFF.getLowerTPS());
    upperMotor.set(TalonFXControlMode.Velocity, ShooterSpeeds.OFF.getUpperTPS());
  }

  public void setSpeed(ShooterSpeeds speeds) {
    lowerMotor.set(TalonFXControlMode.Velocity, lowerMotorController.calculate(getLowerShooterSpeed(), speeds.getLowerTPS()));
    upperMotor.set(TalonFXControlMode.Velocity, upperMotorController.calculate(getUpperShooterSpeed(), speeds.getUpperTPS()));
  }

  public void setUpperShooterSpeed(double upperShooterMotorSpeedTPS) {
    upperMotor.set(TalonFXControlMode.Velocity, upperMotorController.calculate(getUpperShooterSpeed(), upperShooterMotorSpeedTPS));
  }

  public void setLowerShooterSpeed(double lowerShooterMotorSpeedTPS) {
    lowerMotor.set(TalonFXControlMode.Velocity, lowerMotorController.calculate(getLowerShooterSpeed(), lowerShooterMotorSpeedTPS));
  }

  //run this in the perodic, if you don't it wont work.
  public void setShooterRotationAngle(double shooterGoalAngle) {
    shooterGoalAngle = shooterGoalAngle + Constants.Shamper.SHOOTER_ANGLE_OFFSET_IN_DEGREES;

    //for safty. don't want the robot breaking
    if (shooterGoalAngle < Constants.Shamper.Angle.MINIMUM) {
      shooterGoalAngle = Constants.Shamper.Angle.MINIMUM;
    } else if (shooterGoalAngle > Constants.Shamper.Angle.MAXIMUM) {
      shooterGoalAngle = Constants.Shamper.Angle.MAXIMUM;
    }
    //sets movement
    rotationMotor.set(TalonFXControlMode.Position, rotationMotorController.calculate(
    ((getShooterAngle() / 360) * Constants.Shamper.TALONFX_TICS_PER_FULL_ROTATION), 
    ((shooterGoalAngle / 360) * Constants.Shamper.TALONFX_TICS_PER_FULL_ROTATION) * Constants.Shamper.ROTAION_MOTOR_TO_ACTUAL_ROTION_GEAR_RATIO));

  }

  public double getShooterAngle() {
    return rotationEncoder.get() * 360;
  }

  public boolean checkShooterAngleValidity(double testShooterAngle) {
      if (testShooterAngle < Constants.Shamper.Angle.MINIMUM) {
      return false;
    } else if (testShooterAngle > Constants.Shamper.Angle.MAXIMUM) {
      return false;
    }
    return true;
  }

  public double getUpperShooterSpeed() {
    return upperMotor.getSelectedSensorVelocity();
  }

  public double getLowerShooterSpeed() {
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
  
  public static enum ShooterSpeeds {
    OFF(0, 0),
    IDLING(Constants.Shamper.LOWER_SHOOTER_IDLE_SPEED_TPS, Constants.Shamper.UPPER_SHOOTER_IDLE_SPEED_TPS),
    SPEAKER(Constants.Shamper.LOWER_SHOOTER_SPEAKER_SPEED_TPS, Constants.Shamper.UPPER_SHOOTER_SPEAKER_SPEED_TPS),
    AMP(Constants.Shamper.LOWER_SHOOTER_AMP_SPEED_TPS, Constants.Shamper.UPPER_SHOOTER_AMP_SPEED_TPS);

    private final int lowerTicksPerSecond;
    private final int upperTicksPerSecond;

    private ShooterSpeeds(int lowerTicksPerSecond, int upperTicksPerSecond) {
      this.lowerTicksPerSecond = lowerTicksPerSecond;
      this.upperTicksPerSecond = upperTicksPerSecond;
    }

    public int getLowerTPS() {
      return lowerTicksPerSecond;
    }

    public int getUpperTPS() {
      return upperTicksPerSecond;
    }
  }
}