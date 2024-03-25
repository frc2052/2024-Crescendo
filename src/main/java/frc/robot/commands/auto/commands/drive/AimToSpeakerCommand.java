// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commands.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.io.Dashboard;

public class AimToSpeakerCommand extends Command {
    protected final DrivetrainSubsystem drivetrain;
    private final ProfiledPIDController rotationController;
    private boolean isFinished = false;
    private RobotState robotState;

    public AimToSpeakerCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        isFinished = false;
        robotState = RobotState.getInstance();

        rotationController = new ProfiledPIDController(0.25, 0, 0.1, Constants.Drivetrain.AIM_PID_CONSTRAINT);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(0.5);

        addRequirements(drivetrain);
    }

    private double getRotation() {
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

    @Override
    public void execute() {
        drivetrain.drive(0, 0, getRotation(), true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
