// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.io.Dashboard;

public class DriveWhileAimingCommand extends DriveCommand {
    private final ProfiledPIDController rotationController;

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

        rotationController = new ProfiledPIDController(0.5, 0, 0.1, Constants.Drivetrain.AIM_PID_CONSTRAINT);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(3);

        robotState = RobotState.getInstance();
    }
    
    @Override
    protected double getRotation() {
        double goalAngle = AimingCalculator.angleToPoint(robotState.getSpeakerLocation(), robotState.getRobotPose(), robotState.getChassisSpeeds());
        double currentAngle = robotState.getRotation2d180().getRadians();
        Dashboard.getInstance().putData("AIM GOAL ANGLE", goalAngle);
        Dashboard.getInstance().putData("AIM CURRENT ANGLE", currentAngle);
            
        double rotation = rotationController.calculate(currentAngle, goalAngle);
        System.out.println("Rotation " + rotation);
        // do we want to check the error?
        double error = rotationController.getPositionError();

        if(Math.abs(rotation) < 0.03 && rotation != 0){
            // needs at least 5% power to make robot turn
            rotation = Math.copySign(0.03, rotation);
        }
        // double output = MathUtil.clamp(rotation, -DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond() * 2 * Math.PI, DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond()* 2 * Math.PI);
        return rotation;
    }
}
