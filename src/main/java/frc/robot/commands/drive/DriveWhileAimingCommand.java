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

        rotationController = new ProfiledPIDController(0.25, 0, 0.1, Constants.Drivetrain.AIM_PID_CONSTRAINT);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(0.5);

        robotState = RobotState.getInstance();
    }
    
    @Override
    protected double getRotation() {
        // double goalAngleDegrees = AimingCalculator.calculateAngle(RobotState.getInstance().getRobotPose());
        // double deltaDegrees = RobotState.getInstance().getRotation2d360().getDegrees() - goalAngleDegrees;
        // Logger.recordOutput("goal angle", goalAngleDegrees);
        // Logger.recordOutput("measured angle", RobotState.getInstance().getRotation2d360().getDegrees());

        
        // // System.out.println(rotationController.calculate(RobotState.getInstance().getRotation2d360().getDegrees(), goalAngleDegrees) / 360);
        // return rotationController.calculate(RobotState.getInstance().getRotation2d360().getDegrees(), goalAngleDegrees) / 360;
        // // return 0;
        // // if (Math.abs(deltaDegrees) > 90) {
        // //     return Math.copySign(0.5, -deltaDegrees);
        // // } else if (Math.abs(deltaDegrees) > 45){
        // //     return Math.copySign(0.25, -deltaDegrees);
        // // } else if (Math.abs(deltaDegrees) > 5){
        // //     return Math.copySign(0.1, -deltaDegrees);
        // // } else {
        // //     return 0;
        // // }

        double goalAngle = AimingCalculator.angleToPoint(robotState.getSpeakerLocation(), robotState.getRobotPose(), robotState.getChassisSpeeds());
        double currentAngle = robotState.getRotation2d180().getRadians();
        Dashboard.getInstance().putData("AIM GOAL ANGLE", goalAngle);
        Dashboard.getInstance().putData("AIM CURRENT ANGLE", currentAngle);
            
        double rotation = rotationController.calculate(currentAngle, goalAngle);
        // do we want to check the error?
        double error = rotationController.getPositionError();
        double output = MathUtil.clamp(rotation, -DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond() * 2 * Math.PI, DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond()* 2 * Math.PI);
        return output;
    }
}
