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

    public ClimberSubsystem() {
        climberMotor = new TalonFX(Constants.Climber.CLIMBER_MOTOR);
        climberMotor.configFactoryDefault();
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.setInverted(true);
        climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        climberMotor.setSelectedSensorPosition(0, 0, 10);
    }

    public void extend(boolean override) {
        climberMotor.set(ControlMode.MotionMagic, Constants.Climber.CLIMBER_EXTENSION_HEIGHT_TICKS);
    }

    public void retract(boolean override) {
        climberMotor.set(ControlMode.MotionMagic, Constants.Climber.CLIMBER_RETRACTION_HEIGHT_TICKS);
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

    public double getEncoderPosition() {
        return climberMotor.getSelectedSensorPosition();
    }

    public boolean getEncoderIsAbove(double ticks) {
        return climberMotor.getSelectedSensorPosition() >= ticks;
    }

@Override
public void periodic() {
    SmartDashboard.putNumber("Climber Height Ticks", climberMotor.getSelectedSensorPosition());
}

}