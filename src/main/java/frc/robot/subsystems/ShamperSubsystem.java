// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShamperSubsystem extends SubsystemBase {
  private static TalonFX lowerMotor;
  private static TalonFX upperMotor;
  private static CANSparkMax leftPivotMotor;
  private static CANSparkMax rightPivotMotor;

  private final DutyCycleEncoder rotationEncoder;
  private static DigitalInput limitSwitch;
  private static DigitalInput ampHallEffectSensor;
  private static DigitalInput podiumHallEffectSensor;

  private ProfiledPIDController lowerMotorController;
  private ProfiledPIDController upperMotorController;
  private ProfiledPIDController leftPivotMotorController;

  private double goalAngle;
  private ShamperSpeed goalSpeed;

  public ShamperSubsystem() {
    goalSpeed = ShamperSpeed.OFF;
// TODO: make them coast
    lowerMotor = new TalonFX(Constants.CAN.LOWER_SHOOTER_MOTOR_ID);
    lowerMotorController = new ProfiledPIDController(
    Constants.Shamper.LOWER_MOTOR_KP,
    Constants.Shamper.LOWER_MOTOR_KI,
    Constants.Shamper.LOWER_MOTOR_KD, 
    new TrapezoidProfile.Constraints(
    Constants.Shamper.LOWER_MOTOR_MAX_VELOCITY,
    Constants.Shamper.LOWER_MOTOR_MAX_ACCELERATION));

    upperMotor = new TalonFX(Constants.CAN.UPPER_SHOOTER_MOTOR_ID);
    upperMotorController = new ProfiledPIDController(
    Constants.Shamper.UPPER_MOTOR_KP,
    Constants.Shamper.UPPER_MOTOR_KI,
    Constants.Shamper.UPPER_MOTOR_KD,      
    new TrapezoidProfile.Constraints(
    Constants.Shamper.UPPER_MOTOR_MAX_VELOCITY,
    Constants.Shamper.UPPER_MOTOR_MAX_ACCELERATION));

    leftPivotMotor = new CANSparkMax(Constants.CAN.LEFT_PIVOT_SHAMPER_MOTOR_ID, MotorType.kBrushless);
    leftPivotMotorController = new ProfiledPIDController(
    Constants.Shamper.PIVOT_MOTOR_KP,
    Constants.Shamper.PIVOT_MOTOR_KI,
    Constants.Shamper.PIVOT_MOTOR_KD,      
    new TrapezoidProfile.Constraints(
    Constants.Shamper.PIVOT_MOTOR_MAX_VELOCITY,
    Constants.Shamper.PIVOT_MOTOR_MAX_ACCELERATION));

    rightPivotMotor = new CANSparkMax(Constants.CAN.RIGHT_PIVOT_SHAMPER_MOTOR_ID, MotorType.kBrushless);

    rotationEncoder = new DutyCycleEncoder(Constants.Shamper.ROTATION_ENCODER_PIN);
    rotationEncoder.setPositionOffset(Constants.Shamper.ENCODER_OFFSET_DEGREES / 360);

    lowerMotor.setInverted(Constants.Shamper.LOWER_MOTOR_IS_INVERTED);
    upperMotor.setInverted(Constants.Shamper.UPPER_MOTOR_IS_INVERTED);
    leftPivotMotor.setInverted(Constants.Shamper.LEFT_PIVOT_MOTOR_IS_INVERTED);
    rightPivotMotor.follow(leftPivotMotor, true);

    // limitSwitch = new DigitalInput(Constants.Shamper.LIMIT_SWITCH_PIN);
    // ampHallEffectSensor = new DigitalInput(Constants.Shamper.AMP_HALL_EFFECT_PIN);
    // podiumHallEffectSensor = new DigitalInput(Constants.Shamper.PODIUM_HALL_EFFECT_PIN);
  }

  public void stopShooter() {
    lowerMotor.set(ShamperSpeed.OFF.getLowerPCT());
    upperMotor.set(ShamperSpeed.OFF.getUpperPCT());
  }

  public void setShootSpeed(ShamperSpeed speeds) { 
    goalSpeed = speeds;
    lowerMotor.set(lowerMotorController.calculate(getLowerShamperSpeed(), goalSpeed.getLowerPCT()));
    upperMotor.set(-upperMotorController.calculate(getUpperShamperSpeed(), goalSpeed.getUpperPCT()));
  }

  public void setShootSpeed(double lowerSpeed, double upperSpeed) {
    lowerMotor.set(lowerMotorController.calculate(getLowerShamperSpeed(), contrainSpeed(lowerSpeed)));
    upperMotor.set(-upperMotorController.calculate(getUpperShamperSpeed(), contrainSpeed(upperSpeed)));
  }

  public double contrainSpeed(double speed){
    if(speed > 1){
      speed = 1;
    } else if (speed < -1){
      speed = -1;
    }

    return speed;
  }

  public boolean atSpeed(TalonFX motor, double goalSpeed){
    if((Math.abs(motor.get()) - Math.abs(goalSpeed)) < Constants.Shamper.DEAD_ZONE_SHOOTER_SPEED_PCT){
      return true;
    } else {
      return false;
    }
  }

  public boolean shooterAtSpeed(){
    if((atSpeed(lowerMotor, goalSpeed.lowerPercentOutput)) && atSpeed(upperMotor, goalSpeed.upperPercentOutput)){
      return true;
    } else {
      return false;
    }
  }

  public boolean shooterAtSpeed(double goalSpeed){
    if((atSpeed(lowerMotor, goalSpeed)) && atSpeed(upperMotor, goalSpeed)){
      return true;
    } else {
      return false;
    }
  }

  public void setAngle(double goalAngle) {
    this.goalAngle = constrain(goalAngle);
    leftPivotMotor.set(leftPivotMotorController.calculate(getShamperAngle() / 360, goalAngle / 360));
  }

  public void stopPivot() {
    leftPivotMotor.set(0);
    rightPivotMotor.set(0);
  }

  public boolean atAngle() {
    if (goalAngle == Constants.Shamper.Angle.AMP) {
      return isAtAmpLevel();
    } else if (goalAngle == Constants.Shamper.Angle.DEFAULT) {
      return isAtPodiumLevel();
    } else if (goalAngle == getShamperAngle()){
      return true;
    } else {
      return Math.abs(
          getShamperAngle() - goalAngle
      ) <= Constants.Shamper.DEAD_ZONE_DEGREES;
    }
  }

  public double getShamperAngle() {
    return rotationEncoder.getAbsolutePosition() * 360;
  }

  public double constrain(double angle) {
    if (angle < Constants.Shamper.Angle.MINIMUM) {
      angle = Constants.Shamper.Angle.MINIMUM;
    } else if (angle > Constants.Shamper.Angle.MAXIMUM) {
      angle = Constants.Shamper.Angle.MAXIMUM;
    }
    
    return angle;
  }

  public double getUpperShamperSpeed() {
    return upperMotor.get();
  }

  public double getLowerShamperSpeed() {
    return lowerMotor.get();
  }  

  public void manualUp() {
      leftPivotMotor.set(Constants.Shamper.PIVOT_MOTOR_MANUAL_UP_SPEED);
  }

  public void manualDown() {
    if (!shamperZeroed()) {
      leftPivotMotor.set(Constants.Shamper.PIVOT_MOTOR_MANUAL_DOWN_SPEED);
    }
  }

  public static TalonFX getUpperTalonFX() {
    return upperMotor;
  }

  public static TalonFX getLowerTalonFX() {
    return lowerMotor;
  }

  public boolean isAtAmpLevel() {
    //return !ampHallEffectSensor.get();
    return false;
  }

  public boolean isAtPodiumLevel() {
    //return !podiumHallEffectSensor.get();
    return false;
  }

  public boolean shamperZeroed() {
    //return !limitSwitch.get();
    return false;
  }

  @Override
  public void periodic() {
    if(shamperZeroed()){
      rotationEncoder.reset();
    }

    // if (atAngle()) {
    //     stopPivot();
    // } else {
    //   setAngle(goalAngle);
    // }

    if(atSpeed(upperMotor, goalSpeed.getUpperPCT())) {
      SmartDashboard.putBoolean("Upper Motor At Speed", true);
    } else {
      SmartDashboard.putBoolean("Upper Motor At Speed", false);
    }

    if(atSpeed(lowerMotor, goalSpeed.getLowerPCT())) {
      SmartDashboard.putBoolean("Lower Motor At Speed", true);
    } else {
      SmartDashboard.putBoolean("Lower Motor At Speed", false);
    }
  }

  public static enum ShamperSpeed {
    OFF(0, 0, true),
    SPEAKER_IDLE(Constants.Shamper.LOWER_SHAMPER_SPEAKER_IDLE_SPEED_PCT, Constants.Shamper.UPPER_SHAMPER_SPEAKER_IDLE_SPEED_PCT, true),
    AMP_IDLE(Constants.Shamper.LOWER_SHAMPER_AMP_IDLE_SPEED_PCT, Constants.Shamper.UPPER_SHAMPER_AMP_IDLE_SPEED_PCT, true),
    SPEAKER_SCORE(Constants.Shamper.LOWER_SHAMPER_SPEAKER_SPEED_PCT, Constants.Shamper.UPPER_SHAMPER_SPEAKER_SPEED_PCT, false),
    AMP_SCORE(Constants.Shamper.LOWER_SHAMPER_AMP_SPEED_PCT, Constants.Shamper.UPPER_SHAMPER_AMP_SPEED_PCT, false);

    private final double lowerPercentOutput;
    private final double upperPercentOutput;
    private final boolean isIdle;

    private ShamperSpeed(double lowerPercentOutput, double upperPercentOutput, boolean isIdle) {
      this.lowerPercentOutput = lowerPercentOutput;
      this.upperPercentOutput = upperPercentOutput;
      this.isIdle = isIdle;
    }

    public double getLowerPCT() {
      return lowerPercentOutput;
    }

    public double getUpperPCT() {
      return upperPercentOutput;
    }

    public boolean isIdle() {
      return isIdle;
    }
  }
}