// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.MusicPlayerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final static IntakeSubsystem intake = new IntakeSubsystem();
  private final static ShooterSubsystem shooter = new ShooterSubsystem();
  private final static ClimberSubsystem climber = new ClimberSubsystem();
  private final static MusicPlayerSubsystem musicPlayer = new MusicPlayerSubsystem();
  private final static AdvantageScopeSubsystem advantageScope = new AdvantageScopeSubsystem(intake, shooter, climber, drivetrain, musicPlayer);

  private final Joystick joystick = new Joystick(0);
  private final Joystick joystick2 = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    advantageScope.recordData();
            drivetrain.setDefaultCommand(
            new DriveCommand(
                // Forward velocity supplier.
                joystick::getY,
                // Sideways velocity supplier.
                joystick::getX,
                // Rotation velocity supplier.
                joystick2::getX,
                this::isFieldCentric,
                drivetrain
            )
        );

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public Boolean isFieldCentric() {
    return true;
  }
}
