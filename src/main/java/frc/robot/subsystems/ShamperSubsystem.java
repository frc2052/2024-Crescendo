// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShamperSubsystem extends SubsystemBase {
  private static TalonFX lowerMotor;
  private static TalonFX upperMotor;
  private static CANSparkMax leftPivotMotor;
  private static CANSparkMax rightPivotMotor;

  private final DutyCycleEncoder rotationEncoder;
  private static DigitalInput limitSwitch;
  private static DigitalInput AmpHallEffectSensor;
  private static DigitalInput podiumHallEffectSensor;

  private ProfiledPIDController lowerMotorController;
  private ProfiledPIDController upperMotorController;
  private ProfiledPIDController leftPivotMotorController;
  private ProfiledPIDController rightPivotMotorController;

  private boolean limitSwitchTriggered;

  public ShamperSubsystem() {

    lowerMotor = new TalonFX(Constants.Shamper.Motors.LOWER_MOTOR_ID);
    lowerMotorController = new ProfiledPIDController(
    Constants.Shamper.LOWER_MOTOR_KP,
    Constants.Shamper.LOWER_MOTOR_KI,
    Constants.Shamper.LOWER_MOTOR_KD, 
    new TrapezoidProfile.Constraints(
    Constants.Shamper.LOWER_MOTOR_MAX_VELOCITY,
    Constants.Shamper.LOWER_MOTOR_MAX_ACCELERATION));

    upperMotor = new TalonFX(Constants.Shamper.Motors.UPPER_MOTOR_ID);
    upperMotorController = new ProfiledPIDController(
    Constants.Shamper.UPPER_MOTOR_KP,
    Constants.Shamper.UPPER_MOTOR_KI,
    Constants.Shamper.UPPER_MOTOR_KD,      
    new TrapezoidProfile.Constraints(
    Constants.Shamper.UPPER_MOTOR_MAX_VELOCITY,
    Constants.Shamper.UPPER_MOTOR_MAX_ACCELERATION));

    leftPivotMotor = new CANSparkMax(Constants.Shamper.LEFT_PIVOT_SHAMPER_MOTOR_ID, MotorType.kBrushless);
    leftPivotMotorController = new ProfiledPIDController(
    Constants.Shamper.PIVOT_MOTOR_KP,
    Constants.Shamper.PIVOT_MOTOR_KI,
    Constants.Shamper.PIVOT_MOTOR_KD,      
    new TrapezoidProfile.Constraints(
    Constants.Shamper.PIVOT_MOTOR_MAX_VELOCITY,
    Constants.Shamper.PIVOT_MOTOR_MAX_ACCELERATION));

    rightPivotMotor = new CANSparkMax(Constants.Shamper.RIGHT_PIVOT_SHAMPER_MOTOR_ID, MotorType.kBrushless);
    rightPivotMotorController = new ProfiledPIDController(
    Constants.Shamper.PIVOT_MOTOR_KP,
    Constants.Shamper.PIVOT_MOTOR_KI,
    Constants.Shamper.PIVOT_MOTOR_KD,      
    new TrapezoidProfile.Constraints(
    Constants.Shamper.PIVOT_MOTOR_MAX_VELOCITY,
    Constants.Shamper.PIVOT_MOTOR_MAX_ACCELERATION));

    rotationEncoder = new DutyCycleEncoder(Constants.Shamper.ROTATION_ENCODER_ID);

    lowerMotor.setInverted(Constants.Shamper.LOWER_MOTOR_IS_INVERTED);
    upperMotor.setInverted(Constants.Shamper.UPPER_MOTOR_IS_INVERTED);
    leftPivotMotor.setInverted(Constants.Shamper.LEFT_PIVOT_MOTOR_IS_INVERTED);
    rightPivotMotor.setInverted(Constants.Shamper.RIGHT_PIVOT_MOTOR_IS_INVERTED);

    limitSwitch = new DigitalInput(Constants.Shamper.LIMIT_SWITCH_ID);
    AmpHallEffectSensor = new DigitalInput(Constants.Shamper.AMP_HALL_EFFECT_ID);
    podiumHallEffectSensor = new DigitalInput(Constants.Shamper.PODIUM_HALL_EFFECT_ID);

    limitSwitchTriggered = false;
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
    if (!limitSwitchTriggered) {
    leftPivotMotor.set((!limitSwitchTriggered) ? 
    (leftPivotMotorController.calculate(
    ((getShamperAngle() / 360) * Constants.MotorConstants.PIVOT_MOTOR_TICKS_PER_ROTATION) / 
    ((shooterGoalAngle / 360) * Constants.MotorConstants.PIVOT_MOTOR_TICKS_PER_ROTATION) * Constants.Shamper.PIVOT_GEAR_RATIO, 1)) : 0);

    rightPivotMotor.set((!limitSwitchTriggered) ? 
    (rightPivotMotorController.calculate(
    ((getShamperAngle() / 360) * Constants.MotorConstants.PIVOT_MOTOR_TICKS_PER_ROTATION) / 
    ((shooterGoalAngle / 360) * Constants.MotorConstants.PIVOT_MOTOR_TICKS_PER_ROTATION) * Constants.Shamper.PIVOT_GEAR_RATIO, 1)) : 0);
    }
  }

  public double getShamperAngle() {
    return rotationEncoder.getAbsolutePosition() * 360 + Constants.Shamper.SHAMPER_ENCODER_OFFSET_IN_DEGREES;
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

  public void manualUp() {
    leftPivotMotor.set(1);
    rightPivotMotor.set(1);
  }

  public void manualDown() {
    if (!limitSwitchTriggered) {
      leftPivotMotor.set(-1);
      rightPivotMotor.set(-1);
    }
  }

  public void stopManual() {
    leftPivotMotor.stopMotor();
    rightPivotMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if (!limitSwitch.get()) {
      limitSwitchTriggered = true;
    } else {
      limitSwitchTriggered = false;
    }
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

  public static boolean isAtAmpLevel() {
    return !AmpHallEffectSensor.get();
  }

  public static boolean isAtPodiumLevel() {
    return !podiumHallEffectSensor.get();
  }
}