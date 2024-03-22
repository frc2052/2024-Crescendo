// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.AimingCalculator;
import frc.robot.util.io.Dashboard;
import frc.robot.util.io.Dashboard.ShamperIdleMode;

public class ShamperSubsystem extends SubsystemBase {
  private static TalonFX lowerMotor;
  private static TalonFX upperMotor;
  private static CANSparkMax leftPivotMotor;
  private static CANSparkMax rightPivotMotor;

  private SparkPIDController pivotController;
  private SparkAbsoluteEncoder pivotEncoder;

  // private final DutyCycleEncoder rotationEncoder;
  private static DigitalInput limitSwitch;
  private static DigitalInput ampHallEffectSensor;
  private static DigitalInput podiumHallEffectSensor;

  private double goalAngle;
  private double goalSpeedUpper;
  private double goalSpeedLower;
  private ProfiledPIDController pivotMotorController;

  private boolean override;

  private final VelocityVoltage shooterVelocity;

  private final Slot0Configs slot0Configs;

  private ClosedLoopRampsConfigs windDownConfig;
  private ClosedLoopRampsConfigs shootConfig;

  private ShamperIdleMode currentIdleMode;

  public ShamperSubsystem() {
    currentIdleMode = ShamperIdleMode.SPEAKER_IDLE;

    goalSpeedUpper = 0;
    goalSpeedLower = 0;
    goalAngle = Constants.Shamper.Angle.SUB;

    lowerMotor = new TalonFX(Constants.CAN.LOWER_SHOOTER_MOTOR_ID);

    upperMotor = new TalonFX(Constants.CAN.UPPER_SHOOTER_MOTOR_ID);

    leftPivotMotor = new CANSparkMax(Constants.CAN.LEFT_PIVOT_SHAMPER_MOTOR_ID, MotorType.kBrushless);
    rightPivotMotor = new CANSparkMax(Constants.CAN.RIGHT_PIVOT_SHAMPER_MOTOR_ID, MotorType.kBrushless);

    leftPivotMotor.restoreFactoryDefaults();
    rightPivotMotor.restoreFactoryDefaults();

    pivotEncoder = leftPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(360);
    pivotEncoder.setInverted(Constants.Shamper.ENCODER_INVERTED);

    pivotController = leftPivotMotor.getPIDController();
    pivotController.setFeedbackDevice(pivotEncoder);
    pivotController.setP(0.01);
    pivotController.setI(0);
    pivotController.setD(0);
    pivotController.setFF(0.001);
    pivotController.setOutputRange(-1, 1);
    
    leftPivotMotor.setInverted(Constants.Shamper.LEFT_PIVOT_MOTOR_IS_INVERTED);
    rightPivotMotor.setInverted(Constants.Shamper.RIGHT_PIVOT_MOTOR_IS_INVERTED);
    rightPivotMotor.follow(leftPivotMotor, false);

    leftPivotMotor.setIdleMode(IdleMode.kBrake);
    rightPivotMotor.setIdleMode(IdleMode.kBrake);

    // rotationEncoder = new DutyCycleEncoder(Constants.Shamper.ROTATION_ENCODER_PIN);
    // rotationEncoder.setPositionOffset(Constants.Shamper.ENCODER_OFFSET_DEGREES / 360);
    // rotationEncoder.setDistancePerRotation(360);
    
    shooterVelocity = new VelocityVoltage(0);

    // robot init, set slot 0 gains
    slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0.000;
    slot0Configs.kD = 0.0;
    
    windDownConfig = new ClosedLoopRampsConfigs();
    windDownConfig.VoltageClosedLoopRampPeriod = 0.75;
    
    shootConfig = new ClosedLoopRampsConfigs();
    shootConfig.VoltageClosedLoopRampPeriod = 0;
    
    lowerMotor.getConfigurator().apply(slot0Configs, 0.050);
    upperMotor.getConfigurator().apply(slot0Configs, 0.050);

    lowerMotor.setInverted(Constants.Shamper.LOWER_MOTOR_IS_INVERTED);
    upperMotor.setInverted(Constants.Shamper.UPPER_MOTOR_IS_INVERTED);

    lowerMotor.setNeutralMode(NeutralModeValue.Coast);
    upperMotor.setNeutralMode(NeutralModeValue.Coast);

    limitSwitch = new DigitalInput(Constants.Shamper.LIMIT_SWITCH_PIN);
    // ampHallEffectSensor = new DigitalInput(Constants.Shamper.AMP_HALL_EFFECT_PIN);
    // podiumHallEffectSensor = new DigitalInput(Constants.Shamper.PODIUM_HALL_EFFECT_PIN);

    // pivotMotorController = new ProfiledPIDController(
    //   Constants.Shamper.PIVOT_MOTOR_KP,
    //   Constants.Shamper.PIVOT_MOTOR_KI,
    //   Constants.Shamper.PIVOT_MOTOR_KD,      
    //   new TrapezoidProfile.Constraints(
    //   Constants.Shamper.PIVOT_MOTOR_MAX_VELOCITY,
    //   Constants.Shamper.PIVOT_MOTOR_MAX_ACCELERATION));
     
    // pivotMotorController.setTolerance(Constants.Shamper.DEAD_ZONE_DEGREES);
  }

  public void setShootSpeed(ShamperSpeed speeds) { 
    if(speeds.getUpper() == goalSpeedUpper && speeds.getLower() == goalSpeedLower) {
      // do nothing if set speed is already the goal speed
      return;
    }

    if(goalSpeedUpper == 0 || (Math.copySign(1, upperMotor.getVelocity().getValueAsDouble()) == Math.copySign(1, speeds.getUpper()))) {
      // no ramp needed, wheel either stopped or already spinning in same direction
      lowerMotor.getConfigurator().apply(shootConfig, 0.05);
      upperMotor.getConfigurator().apply(shootConfig, 0.05);
    } else {
      // ramp needed, wheel goal is to be reversed
      lowerMotor.getConfigurator().apply(windDownConfig, 0.05);
      upperMotor.getConfigurator().apply(windDownConfig, 0.05);
    }

    goalSpeedUpper = speeds.getUpper();
    goalSpeedLower = speeds.getLower();

    shooterVelocity.Slot = 0;
    lowerMotor.setControl(shooterVelocity.withVelocity(goalSpeedLower));
    upperMotor.setControl(shooterVelocity.withVelocity(goalSpeedUpper));
  }

  public void setShootSpeed(double lowerSpeed, double upperSpeed) {
    goalSpeedLower = lowerSpeed;
    goalSpeedUpper = upperSpeed;
    lowerMotor.getConfigurator().apply(shootConfig, 0.05);
    upperMotor.getConfigurator().apply(shootConfig, 0.05);
    shooterVelocity.Slot = 0;
    lowerMotor.setControl(shooterVelocity.withVelocity(lowerSpeed));
    upperMotor.setControl(shooterVelocity.withVelocity(upperSpeed));
  }

  public void setShootSpeedPct(double lowerSpeedPct, double upperSpeedPct) {
    lowerMotor.getConfigurator().apply(shootConfig, 0.05);
    upperMotor.getConfigurator().apply(shootConfig, 0.05);
    shooterVelocity.Slot = 0;
    lowerMotor.set(contrainSpeed(lowerSpeedPct));
    upperMotor.set(contrainSpeed(upperSpeedPct));
  }

  public double contrainSpeed(double speed){
    if(speed > 1){
      speed = 1;
    } else if (speed < -1){
      speed = -1;
    }

    return speed;
  }

  public void windDownShooter(){
    lowerMotor.getConfigurator().apply(windDownConfig, 0.05);
    upperMotor.getConfigurator().apply(windDownConfig, 0.05);

    lowerMotor.setControl(shooterVelocity.withVelocity(0));
    upperMotor.setControl(shooterVelocity.withVelocity(0));
  }

  public boolean motorAtSpeed(TalonFX motor, double goalSpeed){
    boolean motorAtSpeed = motor.getVelocity().getValueAsDouble() > goalSpeed - Constants.Shamper.DEAD_ZONE_SHOOTER_SPEED_RPS
    && motor.getVelocity().getValueAsDouble() < goalSpeed + Constants.Shamper.DEAD_ZONE_SHOOTER_SPEED_RPS
    && goalSpeed != 0;

    return motorAtSpeed;
  }

  public boolean shooterAtSpeed(double lowerGoalSpeed, double upperGoalSpeed){
    if((motorAtSpeed(lowerMotor, lowerGoalSpeed)) && motorAtSpeed(upperMotor, upperGoalSpeed)){
      return true;
    } else {
      return false;
    }
  }

  public void stopShooter(){
    upperMotor.set(0);
    lowerMotor.set(0);
  }

  // public void setAngle(double goalAngle) {
  //   //System.out.println("Setting Angle " + goalAngle);
  //   this.goalAngle = goalAngle;
  // }

  public void setAngle(double degrees) {
    SmartDashboard.putNumber("Last Requested Angles", degrees);
    if(degrees > Constants.Shamper.Angle.MAXIMUM) {
      degrees = Constants.Shamper.Angle.MAXIMUM;
    } else if (degrees < Constants.Shamper.Angle.MINIMUM) {
      degrees = Constants.Shamper.Angle.MINIMUM;
    }

    pivotController.setReference(degrees, ControlType.kPosition);
  }

  public void runPivot(double pct) {
    leftPivotMotor.set(pct);
    // pct = constrainPivotSpeed(pct); 
    // if(getShamperAngle() > Constants.Shamper.Angle.MINIMUM && getShamperAngle() < Constants.Shamper.Angle.MAXIMUM) {
    //   leftPivotMotor.set(pct);
    // } else if (getShamperAngle() < Constants.Shamper.Angle.MINIMUM && pct > 0) { // If we are below the minimum but trying to go up it's fine
    //   leftPivotMotor.set(pct);      
    // } else if (getShamperAngle() > Constants.Shamper.Angle.MAXIMUM && pct < 0) {// If we are above the maximum but trying to go down it's fine
    //   leftPivotMotor.set(pct);      
    // } else {
    //   leftPivotMotor.stopMotor();
    // }
  }

  public double getPivotSpeed(){
    return leftPivotMotor.get();
  }

  public double constrainPivotSpeed(double speed) {
    if (Math.abs(speed) > Constants.Shamper.PIVOT_MOTOR_MAX_VELOCITY){
      speed = Math.copySign(Constants.Shamper.PIVOT_MOTOR_MAX_VELOCITY, speed);
      return speed;
    } else {
      return speed;
    }
  }

  public void stopPivot() {
    leftPivotMotor.set(0);
    rightPivotMotor.set(0);
  }

  public boolean isAtGoalAngle() {
    return Math.abs(getShamperAngle() - goalAngle) < Constants.Shamper.DEAD_ZONE_DEGREES;
  }

  public double getShamperAngle() {
    // % is "mod", mod will always give remander when dividing by 360
    // subtracting 360 to reverse the angle and getting the absolute value so it is positive

    return pivotEncoder.getPosition();

    // double angle = rotationEncoder.getAbsolutePosition() % 1.0;
    // if (angle < 0) {
    //   angle += 1;
    // }

    // return 360 - (angle * 360);
    // return Math.abs((rotationEncoder.getDistance() % 360) - 360);
  }

  public double constrainAngle(double angle) {
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
    System.out.println("MANUAL UP: " + goalAngle);
    this.goalAngle = this.goalAngle + 1;
  }

  public void manualDown() {
    System.out.println("MANUAL DOWN: " + goalAngle);
    this.goalAngle = this.goalAngle - 1;
  }

  public void resetOverride(){
    override = false;
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
    return !limitSwitch.get();
  }

  public void toggleCurrentIdle() {
    switch (currentIdleMode) {
        // go from speaker idle to amp idle
        case SPEAKER_IDLE:
          currentIdleMode = ShamperIdleMode.AMP_IDLE;
          SmartDashboard.putString(Constants.Dashboard.IDLE_MODE_KEY, currentIdleMode.name());
          break;
        // go from amp idle to speaker idle
        case AMP_IDLE:
          currentIdleMode = ShamperIdleMode.SPEAKER_IDLE;
          SmartDashboard.putString(Constants.Dashboard.IDLE_MODE_KEY, currentIdleMode.name());
          break;
    }
  }

  public ShamperIdleMode getCurrentIdleMode() {
    return currentIdleMode;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("upperGoal", goalSpeedUpper);
    Logger.recordOutput("Shamper Current Angle:  ", getShamperAngle());
    Logger.recordOutput("Shamper Current Goal Angle: ", goalAngle);
    Logger.recordOutput("Shamper Shooter At Speed ", shooterAtSpeed(goalSpeedLower, goalSpeedUpper));
    Logger.recordOutput("Upper Shooter Speed ", upperMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Lower Shooter Speed ", lowerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Current Shamper Angle", getShamperAngle());

    RobotState.getInstance().updateShamperAtGoalAngle(isAtGoalAngle());
    
    // if(!(getPivotSpeed() < 0 && shamperZeroed())) {
    //   if (goalAngle > Constants.Shamper.Angle.MINIMUM && goalAngle < Constants.Shamper.Angle.MAXIMUM) {
    //     double speed = Math.copySign(pivotMotorController.calculate(getShamperAngle(), goalAngle), (goalAngle - getShamperAngle()));
    //   } else {
    //     System.out.println("Goal Angle for Shamper Pivot Invalid: " + goalAngle);
    //     stopPivot();
    //   }

    // if (goalAngle > Constants.Shamper.Angle.MINIMUM && goalAngle < Constants.Shamper.Angle.MAXIMUM) {
    //   if (isAtGoalAngle()) {
    //     stopPivot();
    //     //System.out.println("At Goal of " + this.goalAngle);
    //   } else if (Math.abs(getShamperAngle() - goalAngle) > 10) {
    //     double speed = Math.copySign(Constants.Shamper.PIVOT_MOTOR_MAX_VELOCITY, -(getShamperAngle() - goalAngle));
    //     runPivot(speed);
    //     //System.out.println("Go Fast " + Math.copySign(Constants.Shamper.PIVOT_MOTOR_MAX_VELOCITY, -(getShamperAngle() - goalAngle)));
    //   } else {
    //     double speed = Math.copySign(Constants.Shamper.PIVOT_MOTOR_LEVEL_2_VELOCITY, -(getShamperAngle() - goalAngle));
    //     runPivot(speed);
    //     //System.out.println("Go Slow " + Math.copySign(Constants.Shamper.PIVOT_MOTOR_LEVEL_2_VELOCITY, -(getShamperAngle() - goalAngle)));
    //   }
    // } else {
    //   System.out.println("Goal Angle for Shamper Pivot Invalid: " + goalAngle);
    //   stopPivot();
    // }

    // } else {
    //   System.out.println("***TRYING TO LOWER SHAMPER WHILE ZEROED***");
    //   stopPivot();
    // }

  }

  public static enum ShamperSpeed {
    OFF(0, 0),
    SPEAKER_IDLE(Constants.Shamper.LOWER_SHAMPER_SPEAKER_IDLE_SPEED_RPS, Constants.Shamper.UPPER_SHAMPER_SPEAKER_IDLE_SPEED_RPS),
    SUB(Constants.Shamper.LOWER_SHAMPER_SUB_SPEED_RPS, Constants.Shamper.UPPER_SHAMPER_SUB_SPEED_RPS),
    AMP_IDLE(Constants.Shamper.LOWER_SHAMPER_AMP_IDLE_SPEED_RPS, Constants.Shamper.UPPER_SHAMPER_AMP_IDLE_SPEED_RPS),
    SPEAKER_SCORE(Constants.Shamper.LOWER_SHAMPER_SPEAKER_SPEED_RPS, Constants.Shamper.UPPER_SHAMPER_SPEAKER_SPEED_RPS),
    AMP_SCORE(Constants.Shamper.LOWER_SHAMPER_AMP_SPEED_PCT, Constants.Shamper.UPPER_SHAMPER_AMP_SPEED_PCT),
    TRAP(Constants.Shamper.UPPER_SHAMPER_TRAP_SPEED_RPS, Constants.Shamper.LOWER_SHAMPER_TRAP_SPEED_RPS);

    private final double lowerRPS;
    private final double upperRPS;

    private ShamperSpeed(double lowerRPS, double upperRPS) {
      this.lowerRPS = lowerRPS;
      this.upperRPS = upperRPS;
    }

    public double getLower() {
      return lowerRPS;
    }

    public double getUpper() {
      return upperRPS;
    }
  }
}