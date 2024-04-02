// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.io.Dashboard;

public class DriveWhileAimingCommand extends DriveCommand {
    private final PIDController rotationController;
    private SimpleMotorFeedforward rotationFeedForward;
    protected boolean isOnTarget;

    private RobotState robotState;

    /**
     * @param xSupplier supplier for forward velocity.
     * @param ySupplier supplier for sideways velocity.
     * @param rotationSupplier supplier for angular velocity.
     */
    public DriveWhileAimingCommand(
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier,
        BooleanSupplier fieldCentricSupplier,
        DrivetrainSubsystem drivetrain
    ) {
        super(xSupplier, ySupplier, () -> 0, fieldCentricSupplier, drivetrain);

        rotationController = new PIDController(0.3, 0, 0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(0.087, 0.087);

        rotationFeedForward = new SimpleMotorFeedforward(0.013, 0.013, 0);

        robotState = RobotState.getInstance();
    }

    @Override
    public void initialize() {
        RobotState.getInstance().updateIsHorizontalAiming(true);
    }
    
    @Override
    protected double getRotation() {
        double goalAngle = AimingCalculator.angleToPoint(AimingCalculator.calculateAimPointSpeaker(robotState.getRobotPose()), robotState.getRobotPose(), robotState.getChassisSpeeds());
        double currentAngle = robotState.getRotation2d180().getRadians();
        Dashboard.getInstance().putData("AIM GOAL ANGLE", goalAngle);
        Dashboard.getInstance().putData("AIM CURRENT ANGLE", currentAngle);
            
        double rotation = rotationController.calculate(currentAngle, goalAngle);

        // add our rotation
        rotation = rotation + rotationFeedForward.calculate(rotation);

        /*
         *  static friction is 0.61 voltage
         */

        isOnTarget = Math.abs(currentAngle - goalAngle) < Math.toRadians(Constants.Drivetrain.AIM_TOLERANCE_DEGREES);

        if(isOnTarget){
            RobotState.getInstance().updateRotationOnTarget(true);
        } else {
            RobotState.getInstance().updateRotationOnTarget(false);
        }

        Logger.recordOutput("aim rot", rotation);

        return rotation;
    }

     // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotState.getInstance().updateIsHorizontalAiming(false);
    RobotState.getInstance().updateRotationOnTarget(false);
  }
}
