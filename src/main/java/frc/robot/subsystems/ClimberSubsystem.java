package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

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
        RobotState.getInstance().updateIsClimbing(true);
    }

    public void retract(boolean override) {
        leftClimberMotor.set(-Constants.Climber.CLIMBER_MOTOR_PCT);
    }

    public void retractSlow(boolean override) {
        leftClimberMotor.set(-Constants.Climber.CLIMBER_MOTOR_PCT_SLOW);
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
        return leftClimberMotor.getEncoder().getPosition();
    }

    public boolean getEncoderIsAbove(double ticks) {
        //return leftClimberMotor.getSelectedSensorPosition() >= ticks;
        return false;
    }

@Override
public void periodic() {
    Logger.recordOutput("Climber Encoder Value", leftClimberMotor.getEncoder().getPosition());
}

}