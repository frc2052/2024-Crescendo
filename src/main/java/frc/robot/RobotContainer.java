// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Drivetrain;
import frc.robot.commands.Climb.RaiseClimberCommand;
import frc.robot.commands.Climb.LowerClimberCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveWhileMovingAimingCommand;
import frc.robot.commands.drive.DriveWhileStationaryAimingCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.music.PauseMusicPlayerCommand;
import frc.robot.commands.shooter.ShooterAmpAngleCommand;
import frc.robot.commands.shooter.ShooterAmpSpeedCommand;
import frc.robot.commands.shooter.ShooterDefaultAngleCommand;
import frc.robot.commands.shooter.ShooterIdleSpeedCommand;
import frc.robot.commands.shooter.ShooterSpeakerAngleMovingCommand;
import frc.robot.commands.shooter.ShooterSpeakerAngleStationaryCommand;
import frc.robot.commands.shooter.ShooterSpeakerSpeedCommand;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.MusicPlayerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final static IntakeSubsystem intake = new IntakeSubsystem();
  private final static ShooterSubsystem shooter = new ShooterSubsystem();
  private final static ClimberSubsystem climber = new ClimberSubsystem();
  private final static MusicPlayerSubsystem musicPlayer = new MusicPlayerSubsystem();
  private final static AdvantageScopeSubsystem advantageScope = new AdvantageScopeSubsystem(intake, shooter, climber, drivetrain, musicPlayer);

  private final Joystick translationJoystick;
  private final Joystick rotationJoystick;
  private final Joystick controlPanel;

  private JoystickButton staticAimButton;
  private JoystickButton motionAimButton;

  private JoystickButton raiseClimberButton;
  private JoystickButton lowerClimberButton;

  private JoystickButton intakeInButton;
  private JoystickButton intakeOutButton;
  
  private JoystickButton pauseMusicPlayerButton;

  private JoystickButton ampScoreSetupButton;
  private JoystickButton speakerScoreSetupButton;
  private JoystickButton shooterDefaultButton;

  private BooleanSupplier fieldCentricSupplier;
  private boolean musicOn;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    fieldCentricSupplier = new BooleanSupplier() {
            public boolean getAsBoolean() {return true;}
        };
    musicOn = true;

    // Configure the trigger bindings
    configureBindings();
    configureDiveType();
  }

  private void configureBindings() {
    translationJoystick = new Joystick(0);
    rotationJoystick = new Joystick(1);
    controlPanel = new Joystick(2);

    staticAimButton = new JoystickButton(translationJoystick, 0);
    motionAimButton = new JoystickButton(rotationJoystick, 0);

    raiseClimberButton = new JoystickButton(controlPanel, 12);
    lowerClimberButton = new JoystickButton(controlPanel, 11);

    intakeInButton = new JoystickButton(controlPanel, 10);
    intakeOutButton = new JoystickButton(controlPanel, 9);

    pauseMusicPlayerButton = new JoystickButton(controlPanel, 1);

    ampScoreSetupButton = new JoystickButton(controlPanel, 7);
    speakerScoreSetupButton = new JoystickButton(controlPanel, 6);
    shooterDefaultButton = new JoystickButton(controlPanel, 8);


    raiseClimberButton.whileTrue(new RaiseClimberCommand(climber));
    lowerClimberButton.whileTrue(new LowerClimberCommand(climber));

    intakeInButton.whileTrue(new IntakeInCommand(intake));
    intakeOutButton.whileTrue(new IntakeOutCommand(intake));

    pauseMusicPlayerButton.onTrue(toggleMusic());

    ampScoreSetupButton.whileTrue(ampSetup());
    speakerScoreSetupButton.whileTrue(speakerSetup());
    shooterDefaultButton.onTrue(defaultSetup());
  }

  private void configureDiveType() {
    normalDrive();
    staticAimButton.onTrue(staticAimDrive());
    motionAimButton.onTrue(motionAimDrive());
    staticAimButton.onFalse(normalDrive());
    staticAimButton.onFalse(normalDrive());
  }

  private Command staticAimDrive() {
    drivetrain.setDefaultCommand(
      new RunCommand(() -> new DriveWhileStationaryAimingCommand(fieldCentricSupplier, drivetrain)));
    return null;
  }

  private Command motionAimDrive() {
    drivetrain.setDefaultCommand(
      new RunCommand(() -> new DriveWhileMovingAimingCommand(() -> translationJoystick.getX(), () -> translationJoystick.getY(), fieldCentricSupplier, drivetrain)));
    return null;
  }

  private Command normalDrive() {
    drivetrain.setDefaultCommand(
      new RunCommand(() -> new DriveCommand(() -> translationJoystick.getX(), () -> translationJoystick.getY(), () -> rotationJoystick.getDirectionDegrees(), fieldCentricSupplier, drivetrain)));
    return null;
  }

  public Command toggleMusic() {
    musicOn = !musicOn;
    if (!musicOn) {new PauseMusicPlayerCommand(musicPlayer);}
    return null;
  }

  public Command ampSetup() {
    new ShooterAmpAngleCommand(shooter);
    new ShooterAmpSpeedCommand(shooter);
    return null;
  }

  public Command speakerSetup() {
    new ShooterSpeakerSpeedCommand(shooter);
    if (staticAimButton.getAsBoolean()) {
      new ShooterSpeakerAngleStationaryCommand(shooter);
    } else {
      new ShooterSpeakerAngleMovingCommand(shooter);
    }
    return null;
  }

  public Command defaultSetup() {
    new ShooterDefaultAngleCommand(shooter);
    new ShooterIdleSpeedCommand(shooter);
    return null;
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
