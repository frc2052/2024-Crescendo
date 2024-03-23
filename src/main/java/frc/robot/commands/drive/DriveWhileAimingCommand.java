// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AimingCalculator;

public class DriveWhileAimingCommand extends Command {
    protected final DrivetrainSubsystem drivetrain;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final BooleanSupplier fieldCentricSupplier;
    
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final PIDController rotationController;

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
        this.drivetrain = drivetrain;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.fieldCentricSupplier = fieldCentricSupplier;

        rotationController = new PIDController(0.025, 0, 1);
        rotationController.setTolerance(2);

        xLimiter = new SlewRateLimiter(2);
        yLimiter = new SlewRateLimiter(2);

        addRequirements(drivetrain);
    }

    protected double getX() {
        return slewAxis(xLimiter, deadBand(-xSupplier.getAsDouble()));
    }

    protected double getY() {
        return slewAxis(yLimiter, deadBand(-ySupplier.getAsDouble()));
    }

    private double getRotation() {
        double goalAngleDegrees = AimingCalculator.calculateAngle(RobotState.getInstance().getRobotPose());
        double deltaDegrees = RobotState.getInstance().getRotation2d360().getDegrees() - goalAngleDegrees;
        Logger.recordOutput("goal angle", goalAngleDegrees);
        Logger.recordOutput("measured angle", RobotState.getInstance().getRotation2d360().getDegrees());

        
        System.out.println(rotationController.calculate(RobotState.getInstance().getRotation2d360().getDegrees(), goalAngleDegrees) / 360);
        return rotationController.calculate(RobotState.getInstance().getRotation2d360().getDegrees(), goalAngleDegrees);
        // return 0;
        // if (Math.abs(deltaDegrees) > 90) {
        //     return Math.copySign(0.5, -deltaDegrees);
        // } else if (Math.abs(deltaDegrees) > 45){
        //     return Math.copySign(0.25, -deltaDegrees);
        // } else if (Math.abs(deltaDegrees) > 5){
        //     return Math.copySign(0.1, -deltaDegrees);
        // } else {
        //     return 0;
        // }
    }

    @Override
    public void execute() {
        drivetrain.drive(getX(), getY(), getRotation(), true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    protected double slewAxis(SlewRateLimiter limiter, double value) {
        return limiter.calculate(Math.copySign(Math.pow(value, 2), value));
    }

    protected double deadBand(double value) {
        if (Math.abs(value) <= 0.075) {
            return 0.0;
        }
        // Limit the value to always be in the range of [-1.0, 1.0]
        return Math.copySign(Math.min(1.0, Math.abs(value)), value);
    }
}
