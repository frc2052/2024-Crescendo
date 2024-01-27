// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OneUnderBumperIntake;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.MusicPlayerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.intake.OneUnderBumperIntakeSubsystem;
import frc.robot.subsystems.intake.OverBumperIntakeSubsystem;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  public final static OneUnderBumperIntakeSubsystem oneUnderBumperIntakeSubsystem = new OneUnderBumperIntakeSubsystem();
  public final static OverBumperIntakeSubsystem overBumperIntakeSubsystem = new OverBumperIntakeSubsystem();
  public final static VerticalShooterSubsystem verticalShooterSubsystem = new VerticalShooterSubsystem();
  public final static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public final static MusicPlayerSubsystem musicPlayerSubsystem = new MusicPlayerSubsystem();
  private final static AdvantageScopeSubsystem advantageScopeSubsystem = new AdvantageScopeSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
