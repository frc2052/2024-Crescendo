// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.climb.ClimberRetractCommand;
import frc.robot.commands.climb.ClimerExtendCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveWhileMovingAimingCommand;
import frc.robot.commands.drive.DriveWhileOrbitingNoteCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.music.PauseMusicPlayerCommand;
import frc.robot.commands.music.PlayActivationJingleCommand;
import frc.robot.states.Superstructure;
import frc.robot.states.Superstructure.SuperstructureState;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.MusicPlayerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.TrapArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.RobotStatusCommunicator;
import frc.robot.util.io.Dashboard;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final DrivetrainSubsystem drivetrain;
  private final IntakeSubsystem intake;
  private final ShamperSubsystem shamper;
  private final ClimberSubsystem climber;
  private final MusicPlayerSubsystem musicPlayer;
  private final VisionSubsystem vision;
  private final AdvantageScopeSubsystem advantageScope;
  private final IndexerSubsystem indexer;
  private final TrapArmSubsystem trapArm;

  private final Superstructure superstructure;

  private final Joystick translationJoystick;
  private final Joystick rotationJoystick;
  private final Joystick controlPanel;

  private BooleanSupplier fieldCentricSupplier;
  public static boolean musicOn;
  public RobotStatusCommunicator robotStatusCommunicator;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    translationJoystick = new Joystick(0);
    rotationJoystick = new Joystick(1);
    controlPanel = new Joystick(2);

    drivetrain = new DrivetrainSubsystem();
    intake = new IntakeSubsystem();
    shamper = new ShamperSubsystem();
    climber = new ClimberSubsystem();
    musicPlayer = new MusicPlayerSubsystem();
    vision = new VisionSubsystem();
    indexer = new IndexerSubsystem();
    trapArm = new TrapArmSubsystem();
    advantageScope = new AdvantageScopeSubsystem(intake, shamper, climber, drivetrain, musicPlayer, vision, indexer, trapArm);

    superstructure = new Superstructure(shamper, climber, indexer);

    robotStatusCommunicator = new RobotStatusCommunicator(musicPlayer);

    musicOn = true;

    drivetrain.setDefaultCommand(
      new DriveCommand(
          // Forward velocity supplier.
          translationJoystick::getY,
          // Sideways velocity supplier.
          translationJoystick::getX,
          // Rotation velocity supplier.
          rotationJoystick::getX,
          Dashboard.getInstance()::isFieldCentric,
          drivetrain
      )
    );

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
    
    intakeInButton.whileTrue(new IntakeInCommand(intake, indexer));
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
    // An example command will be run in autonomous
    return null;
  }
}
