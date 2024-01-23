package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;

public class FlatShooterSubsystem extends SubsystemBase{
  private final TalonFX flatMotor;

  public FlatShooterSubsystem() {
    flatMotor = new TalonFX(Constants.FlatShooter.FLAT_SHOOTER_MOTOR_ID);
    flatMotor.set(TalonFXControlMode.Velocity, Constants.FlatShooter.SHOOTER_IDLE_SPEED_TPS);
  }

  public void set(double flatShooterMotorSpeedTPS) {
    flatMotor.set(TalonFXControlMode.Velocity, flatShooterMotorSpeedTPS);
  }

  public void stop() {
    flatMotor.set(TalonFXControlMode.Velocity, 0);
  }

  public void idle() {
    flatMotor.set(TalonFXControlMode.Velocity, Constants.FlatShooter.SHOOTER_IDLE_SPEED_TPS);
  }
  public double getSpeed() {
    return flatMotor.getSelectedSensorVelocity();
  }
    @Override
  public void periodic() {}
}
