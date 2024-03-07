// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commands.drive;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AimingCalculator;

public class AimToSpeakerCommand extends Command {
    protected final DrivetrainSubsystem drivetrain;
    private double goalAngleDegrees;
    private double deltaDegrees;
    private boolean isFinished = false;

    public AimToSpeakerCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        isFinished = false;

        addRequirements(drivetrain);
    }

    private double getRotation() {
        isFinished = false;
        goalAngleDegrees = AimingCalculator.calculateAngle(RobotState.getInstance().getRobotPose());
        deltaDegrees = RobotState.getInstance().getRotation2d360().getDegrees() - goalAngleDegrees;
        Logger.recordOutput("goal angle", goalAngleDegrees);
        Logger.recordOutput("measured angle", RobotState.getInstance().getRotation2d360().getDegrees());

        // return 0;
        if (Math.abs(deltaDegrees) > 90) {
            return Math.copySign(0.5, -deltaDegrees);
        } else if (Math.abs(deltaDegrees) > 45){
            return Math.copySign(0.25, -deltaDegrees);
        } else if (Math.abs(deltaDegrees) > 5){
            return Math.copySign(0.1, -deltaDegrees);
        } else {
            isFinished = true;
            return 0;
        }
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
