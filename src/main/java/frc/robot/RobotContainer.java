// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.climb.ClimberRetractCommand;
import frc.robot.commands.climb.ClimerExtendCommand;
import frc.robot.commands.drive.DriveWhileMovingAimingCommand;
import frc.robot.commands.drive.DriveWhileOrbitingNoteCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.music.PauseMusicPlayerCommand;
import frc.robot.states.Superstructure;
import frc.robot.states.Superstructure.SuperstructureState;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.MusicPlayerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final DrivetrainSubsystem drivetrain;
  private final IntakeSubsystem intake;
  private final ShamperSubsystem shamper;
  private final ClimberSubsystem climber;
  private final MusicPlayerSubsystem musicPlayer;
  private final VisionSubsystem vision;
  private final AdvantageScopeSubsystem advantageScope;

  private final Superstructure superstructure;

  private final Joystick translationJoystick;
  private final Joystick rotationJoystick;
  private final Joystick controlPanel;

  private BooleanSupplier fieldCentricSupplier;
  private boolean musicOn;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain = new DrivetrainSubsystem();
    intake = new IntakeSubsystem();
    shamper = new ShamperSubsystem();
    climber = new ClimberSubsystem();
    musicPlayer = new MusicPlayerSubsystem();
    vision = new VisionSubsystem();
    advantageScope = new AdvantageScopeSubsystem(intake, shamper, climber, drivetrain, musicPlayer, vision);
    superstructure = new Superstructure(shamper, climber);

    translationJoystick = new Joystick(0);
    rotationJoystick = new Joystick(0);
    controlPanel = new Joystick(0);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    /*
     * 
     */
    JoystickButton motionAimButton = new JoystickButton(rotationJoystick, 0);
    JoystickButton orbitNoteButton = new JoystickButton(translationJoystick, 0);

    motionAimButton.whileTrue(new DriveWhileMovingAimingCommand(
      () -> translationJoystick.getX(), 
      () -> translationJoystick.getY(), 
      fieldCentricSupplier, 
      drivetrain
    ));

    orbitNoteButton.whileTrue(new DriveWhileOrbitingNoteCommand(
      () -> translationJoystick.getX(), 
      () -> translationJoystick.getY(), 
      fieldCentricSupplier, 
      drivetrain, vision
    ));

    JoystickButton raiseClimberButton = new JoystickButton(controlPanel, 12);
    JoystickButton lowerClimberButton = new JoystickButton(controlPanel, 11);
    
    raiseClimberButton.whileTrue(new ClimerExtendCommand(climber));
    lowerClimberButton.whileTrue(new ClimberRetractCommand(climber));
    /*
     * Intake Button Bindings
     */
    JoystickButton intakeInButton = new JoystickButton(controlPanel, 10);
    JoystickButton intakeOutButton = new JoystickButton(controlPanel, 9);
    
    intakeInButton.whileTrue(new IntakeInCommand(intake));
    intakeOutButton.whileTrue(new IntakeOutCommand(intake));

    JoystickButton toggleMusicPlayerButton = new JoystickButton(controlPanel, 2);
    toggleMusicPlayerButton.onTrue(toggleMusic());

    JoystickButton ampIdleButton = new JoystickButton(controlPanel, 7);
    JoystickButton ampScoreButton = new JoystickButton(controlPanel, 1);
    JoystickButton speakerIdleSetupButton = new JoystickButton(controlPanel, 5);
    JoystickButton speakerScoreSetupButton = new JoystickButton(controlPanel, 6);
    JoystickButton shamperDefaultButton = new JoystickButton(controlPanel, 12);

    ampIdleButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.AMP_IDLE)));
    ampScoreButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.AMP_SCORE)));
    speakerIdleSetupButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.SPEAKER_IDLE)));
    speakerScoreSetupButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.SPEAKER_SCORE)));
    shamperDefaultButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.DEFAULT)));
  }

  public Command toggleMusic() {
    musicOn = !musicOn;
    if (!musicOn) {new PauseMusicPlayerCommand(musicPlayer);}
    return null;
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
