package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
 
    private final TalonFX leftClimberMotor;
    private final TalonFX rightClimberMotor;

    public ClimberSubsystem() {
        leftClimberMotor = new TalonFX(Constants.CAN.LEFT_CLIMBER_MOTOR);
        rightClimberMotor = new TalonFX(Constants.CAN.RIGHT_CLIMBER_MOTOR);

        leftClimberMotor.configFactoryDefault();
        leftClimberMotor.setNeutralMode(NeutralMode.Brake);
        leftClimberMotor.setInverted(true);
        leftClimberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        leftClimberMotor.setSelectedSensorPosition(0, 0, 10);

        rightClimberMotor.configFactoryDefault();
        rightClimberMotor.setNeutralMode(NeutralMode.Brake);
        rightClimberMotor.setInverted(true);
        rightClimberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        rightClimberMotor.setSelectedSensorPosition(0, 0, 10);

        rightClimberMotor.follow(leftClimberMotor);
    }

    public void extend(boolean override) {
        leftClimberMotor.set(ControlMode.MotionMagic, Constants.Climber.CLIMBER_EXTENSION_HEIGHT_TICKS);
    }

    public void retract(boolean override) {
        leftClimberMotor.set(ControlMode.MotionMagic, Constants.Climber.CLIMBER_RETRACTION_HEIGHT_TICKS);
    }

    /**
     * Stops all climber motor activity.
     */
    public void stop() {
        leftClimberMotor.set(ControlMode.PercentOutput, 0);
    }

    public void zeroEncoder() {
        leftClimberMotor.setSelectedSensorPosition(0);
    }

    public double getEncoderPosition() {
        return leftClimberMotor.getSelectedSensorPosition();
    }

    public boolean getEncoderIsAbove(double ticks) {
        return leftClimberMotor.getSelectedSensorPosition() >= ticks;
    }

@Override
public void periodic() {
    SmartDashboard.putNumber("Climber Height Ticks", leftClimberMotor.getSelectedSensorPosition());
}

}