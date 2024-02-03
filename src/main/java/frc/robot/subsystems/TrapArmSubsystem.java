package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TrapArm;

public class TrapArmSubsystem extends SubsystemBase{
    private final TalonFX armMotor;
    private final TalonFX outtakeMotor;

    private final ProfiledPIDController armPIDController;
    private final ProfiledPIDController outtakePIDController;

    public TrapArmSubsystem() {
        armMotor = new TalonFX(Constants.TrapArm.ARM_MOTOR_ID);
        outtakeMotor = new TalonFX(Constants.TrapArm.ARM_ROTATION_MOTOR_ID);

        armPIDController = new ProfiledPIDController(
            Constants.TrapArm.ARM_KP,
            Constants.TrapArm.ARM_KI,
            Constants.TrapArm.ARM_KD, 
            new TrapezoidProfile.Constraints(
            Constants.TrapArm.ARM_MAX_VELOCITY,
            Constants.TrapArm.ARM_MAX_ACCELERATION));

        outtakePIDController = new ProfiledPIDController(
            Constants.TrapArm.OUTTAKE_KP, 
            Constants.TrapArm.OUTTAKE_KI, 
            Constants.TrapArm.ARM_KD, 
            new TrapezoidProfile.Constraints(
            Constants.TrapArm.OUTTAKE_MAX_VELOCITY,
            Constants.TrapArm.OUTTAKE_MAX_ACCELERATION));
    }

    @Override
    public void periodic() {}

    public void setPosition(TrapArmPositions position) {
        armMotor.set(TalonFXControlMode.Position, armPIDController.calculate(armMotor.getSelectedSensorPosition(), position.getPositionTicks()));
    }

    public void setPosition(double position) {
        armMotor.set(TalonFXControlMode.Position, armPIDController.calculate(armMotor.getSelectedSensorPosition(), position));
    }

    public void outtakeOut() {
        outtakeMotor.set(TalonFXControlMode.Velocity, outtakePIDController.calculate(outtakeMotor.getSelectedSensorVelocity(), Constants.TrapArm.OUTTAKE_OUT_SPEED));
    }

    public void outtakeStop() {
        outtakeMotor.set(TalonFXControlMode.Velocity, 0);
    }

    public double getPosition() {
        return armMotor.getSelectedSensorPosition();
    }

    public Boolean atTargetPosition() {
        return true;
    }

    public static enum TrapArmPositions {
        RESTING(0),
        RAISED(25000);
      
        private final double positionTicks;
      
        private TrapArmPositions(double positionTicks) {
            this.positionTicks = positionTicks;
        }
      
        public double getPositionTicks() {
            return positionTicks;
        }
      }
}
