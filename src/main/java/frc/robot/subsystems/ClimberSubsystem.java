package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.io.Dashboard;

public class ClimberSubsystem extends SubsystemBase{
 
    private final CANSparkMax leftClimberMotor;
    private final CANSparkMax rightClimberMotor;

    private final DigitalInput limitSwitch;

    public ClimberSubsystem() {
        leftClimberMotor = new CANSparkMax(Constants.CAN.LEFT_CLIMBER_MOTOR, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.CAN.RIGHT_CLIMBER_MOTOR, MotorType.kBrushless);

        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        leftClimberMotor.setInverted(Constants.Climber.RIGHT_CLIMBER_MOTOR_INVERTED);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setInverted(Constants.Climber.LEFT_CLIMBER_MOTOR_INVERTED);

        rightClimberMotor.follow(leftClimberMotor);

        limitSwitch = new DigitalInput(Constants.Climber.CLIMBER_LIMIT_SWITCH_PIN);
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

    public boolean limitSwitchHit(){
        return !limitSwitch.get();
    }

@Override
public void periodic() {
    Logger.recordOutput("Climber Encoder Value", leftClimberMotor.getEncoder().getPosition());

    if (Dashboard.getInstance().isClimberCoast()){
        leftClimberMotor.setIdleMode(IdleMode.kCoast);
        rightClimberMotor.setIdleMode(IdleMode.kCoast);
    } else {
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
    }
}

}