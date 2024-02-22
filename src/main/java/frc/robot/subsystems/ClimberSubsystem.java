package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
 
    private final CANSparkMax leftClimberMotor;
    private final CANSparkMax rightClimberMotor;

    public ClimberSubsystem() {
        leftClimberMotor = new CANSparkMax(Constants.CAN.LEFT_CLIMBER_MOTOR, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.CAN.RIGHT_CLIMBER_MOTOR, MotorType.kBrushless);

        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        leftClimberMotor.setInverted(Constants.Climber.RIGHT_CLIMBER_MOTOR_INVERTED);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setInverted(Constants.Climber.LEFT_CLIMBER_MOTOR_INVERTED);

        rightClimberMotor.follow(leftClimberMotor);
    }

    public void extend(boolean override) {
        leftClimberMotor.set(Constants.Climber.CLIMBER_MOTOR_PCT);
    }

    public void retract(boolean override) {
        leftClimberMotor.set(-Constants.Climber.CLIMBER_MOTOR_PCT);
    }

    /**
     * Stops all climber motor activity.
     */
    public void stop() {
        leftClimberMotor.set(0);
    }

    public void zeroEncoder() {
        //leftClimberMotor.setSelectedSensorPosition(0);
    }

    public double getEncoderPosition() {
        //return leftClimberMotor.getSelectedSensorPosition();
        return 0;
    }

    public boolean getEncoderIsAbove(double ticks) {
        //return leftClimberMotor.getSelectedSensorPosition() >= ticks;
        return false;
    }

@Override
public void periodic() {
    //SmartDashboard.putNumber("Climber Height Ticks", leftClimberMotor.getSelectedSensorPosition());
}

}