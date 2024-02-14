// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AimingCalculator;

public class DriveWhileAimingCommand extends Command {
    protected final DrivetrainSubsystem drivetrain;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final BooleanSupplier fieldCentricSupplier;
    
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

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

    @Override
    public void execute() {
        drivetrain.drive(getX(), getY(), AimingCalculator.calculate().getRobotAngle(), fieldCentricSupplier.getAsBoolean());
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
