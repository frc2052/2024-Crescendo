// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commands.drive;

import frc.robot.commands.drive.DriveWhileAimingCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AimToSpeakerCommand extends DriveWhileAimingCommand {
    public AimToSpeakerCommand(DrivetrainSubsystem drivetrain) {
        super(()-> 0, ()-> 0, () -> false, drivetrain);

        addRequirements(drivetrain);
    }

    @Override
    public boolean isFinished(){
        if(super.isOnTarget){
            return true;
        } else {
            return false;
        }
    }
}
