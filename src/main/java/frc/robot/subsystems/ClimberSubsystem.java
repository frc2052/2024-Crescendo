package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
 
    // Motor that controls the wenches for climbing.
    private final TalonFX climberMotor;
    private double targetHeightTicks;

    private boolean isVertical;

    public ClimberSubsystem() {
        climberMotor = new TalonFX(Constants.Climber.CLIMBER_MOTOR);
        climberMotor.configFactoryDefault();
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.setInverted(true);
        climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        climberMotor.setSelectedSensorPosition(0, 0, 10);
        // climberMotor.config_kP(0, 3, 0); // Attempts to control climber using MotionMagic
        // climberMotor.config_kI(0, 0, 0);
        // climberMotor.config_kD(0, 0, 0);
        // climberMotor.configMotionCruiseVelocity(100, 10);
        // climberMotor.configMotionAcceleration(100, 10);

       
    }

    /**
     * Extends the climbing arm at a set speed - old way we did it
     */
    public void extend(double extendPctOutput, boolean override) {
     
        if (override) {
        climberMotor.set(ControlMode.PercentOutput, extendPctOutput * 0.5);
        } else if (getIsAboveMaxHeight()) {
            climberMotor.set(ControlMode.PercentOutput, 0);;
        } else if (getIsAbove95PctHeight()) {
            climberMotor.set(ControlMode.PercentOutput, extendPctOutput * 0.5);
        } else {
            // System.err.println("Climber extended to (or past) the max height!");
            climberMotor.set(ControlMode.PercentOutput, extendPctOutput);
        }
    }

    public void extend(boolean override) {
        extend(Constants.Climber.CLIMBER_EXTENSION_SPEED_PCT, override);
    }

    /**
     * Retracts the climbing arm at a set speed
     */
    public void retract(double retractPctOutput, boolean override) {
       
        if (override) {
            climberMotor.set(ControlMode.PercentOutput, retractPctOutput * .75); //slower climb speed when doing override
        } else if (getIsBelow3PctHeight() && getIsAboveMinHeight()) {
            climberMotor.set(ControlMode.PercentOutput, retractPctOutput * 0.5);
        } else if (getIsAboveMinHeight()) {
            climberMotor.set(ControlMode.PercentOutput, retractPctOutput);
        } else {
            // System.err.println("Climber retracted to (or past) the min height!");
            climberMotor.set(ControlMode.PercentOutput, 0);;
        }

    }

    public void retract(boolean override) {
        retract(Constants.Climber.CLIMBER_EXTENSION_SPEED_PCT, override);
    }

    public void movePctOutput(double pctOutput) {
        climberMotor.set(ControlMode.PercentOutput, pctOutput);
        
    }

    public void moveToHeight(double targetHeightTicks) {
        this.targetHeightTicks = targetHeightTicks;
        climberMotor.set(ControlMode.MotionMagic, targetHeightTicks);
    
    }

    /**
     * Stops all climber motor activity.
     */
    public void stop() {
        climberMotor.set(ControlMode.PercentOutput, 0);
    }


    public void zeroEncoder() {
        climberMotor.setSelectedSensorPosition(0);
    }

    public boolean getIsVertical() {
        return isVertical;
    }

    public boolean getIsAboveMaxHeight() {
        return climberMotor.getSelectedSensorPosition() >= (isVertical ? Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_VERTICAL : Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_TILTED);
    }

    public boolean getIsAbove95PctHeight() {
        return climberMotor.getSelectedSensorPosition() >= (isVertical ? Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_VERTICAL : Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_TILTED) * 0.95;
    }

    public boolean getIsBelow3PctHeight() {
        return climberMotor.getSelectedSensorPosition() <= (isVertical ? Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_VERTICAL : Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_TILTED) * 0.03;
    }

    public boolean getIsAboveMinHeight() {
        return climberMotor.getSelectedSensorPosition() >= Constants.Climber.MIN_CLIMBER_HEIGHT_TICKS;
    }

    public double getEncoderPosition() {
        return climberMotor.getSelectedSensorPosition();
    }

    public boolean getEncoderIsAbove(double ticks) {
        return climberMotor.getSelectedSensorPosition() >= ticks;
    }

    public void setArmPostionInches(double inches){
        double ticks = heightInchesToTicks(inches);
        climberMotor.set(ControlMode.Position, ticks);
        targetHeightTicks = ticks;
    }

    public void setTargetClimberHeight(double targetHeightTicks) {
        this.targetHeightTicks = targetHeightTicks;
    }

    public boolean isAtDesiredPosition(){
        return Math.abs(targetHeightTicks - climberMotor.getSelectedSensorPosition()) < 1000;
    }

    private double heightInchesToTicks(double inches) {
        double numberOfRotaions = inches / Constants.Climber.WINCH_CIRCUMFERENCE_INCHES;
        return numberOfRotaions * Constants.Climber.TICKS_PER_WINCH_ROTATION;
    }

@Override
public void periodic() {
    SmartDashboard.putNumber("Climber Height Ticks", climberMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Climber Is At Max Height", getIsAboveMaxHeight());
    SmartDashboard.putBoolean("Climber Is At Min Height", !getIsAboveMinHeight());
}

}