// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        rotationFeedForward = new SimpleMotorFeedforward(0.008, 0, 0);

        robotState = RobotState.getInstance();
    }
    
    @Override
    protected double getRotation() {
        double goalAngle = AimingCalculator.angleToPoint(robotState.getSpeakerLocation(), robotState.getRobotPose(), robotState.getChassisSpeeds());
        double currentAngle = robotState.getRotation2d180().getRadians();
        Dashboard.getInstance().putData("AIM GOAL ANGLE", goalAngle);
        Dashboard.getInstance().putData("AIM CURRENT ANGLE", currentAngle);
            
        double rotation = rotationController.calculate(currentAngle, goalAngle);

        // add our rotation
        rotation = rotation + rotationFeedForward.calculate(rotation);
        // System.out.println("Rotation " + rotation);
        // do we want to check the error?
        double error = rotationController.getPositionError();
        // System.out.println("ERROR: " + error);

        /*
         *  static friction is 0.61 voltage
         *          */

        if(Math.abs(rotation) < 0.04){
            // needs at least 4% power to make robot turn
            rotation = Math.copySign(0.04, rotation);
        } else if (Math.abs(rotation) > 0.4) {
            rotation = Math.copySign(0.4, rotation);
        } 
        // else if (Math.abs(rotation) < 0.05){
        //     rotation = 0;
        // }

        isOnTarget = rotationController.atSetpoint();

        if(isOnTarget){
            rotation = 0;
        }

        // isOnTarget = Math.abs(goalAngle - currentAngle) <= Math.toRadians(3.5);
        System.out.println("on target " + isOnTarget);
        Logger.recordOutput("aim rot", rotation);

        //rotation = rotationSlew.calculate(rotation);
        // double output = MathUtil.clamp(rotation, -DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond() * 2 * Math.PI, DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond()* 2 * Math.PI);
        return rotation;
    }
}
