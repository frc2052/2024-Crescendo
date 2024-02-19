// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.AutoFactory;
import frc.robot.commands.climb.ClimberRetractCommand;
import frc.robot.commands.climb.ClimberExtendCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveWhileAimingCommand;
import frc.robot.commands.drive.DriveWhileOrbitingNoteCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.music.PauseMusicPlayerCommand;
import frc.robot.commands.music.PlayActivationJingleCommand;
import frc.robot.commands.shamper.ShamperManualDownCommand;
import frc.robot.commands.shamper.ShamperManualUpCommand;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.MusicPlayerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TrapArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.util.RobotStatusCommunicator;
import frc.robot.util.io.Dashboard;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private RobotState robotState = RobotState.getInstance();
  private final DrivetrainSubsystem drivetrain;
  private final IntakeSubsystem intake;
  private final ShamperSubsystem shamper;
  private final IndexerSubsystem indexer;
  // private final ClimberSubsystem climber;
  // private final MusicPlayerSubsystem musicPlayer;
  // private final VisionSubsystem vision;
  // private final AdvantageScopeSubsystem advantageScope;
  // private final TrapArmSubsystem trapArm;

  private final Superstructure superstructure;
  public final RobotStatusCommunicator robotStatusCommunicator;

  private final AutoFactory autoFactory;

  private final Joystick translationJoystick;
  private final Joystick rotationJoystick;
  private final Joystick controlPanel;

  private BooleanSupplier fieldCentricSupplier;
  public boolean musicOn;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain = new DrivetrainSubsystem();
    intake = new IntakeSubsystem();
    indexer = new IndexerSubsystem();
    shamper = new ShamperSubsystem();
    indexer = new IndexerSubsystem();
    // climber = new ClimberSubsystem();
    // musicPlayer = new MusicPlayerSubsystem();
    // vision = new VisionSubsystem();
    // trapArm = new TrapArmSubsystem();
    //advantageScope = new AdvantageScopeSubsystem(intake, shamper, climber, drivetrain, musicPlayer, vision, indexer, trapArm);

    // superstructure = new Superstructure(shamper, climber, indexer);
    superstructure = new Superstructure(shamper, indexer);

    //robotStatusCommunicator = new RobotStatusCommunicator(musicPlayer);

    musicOn = true;

    autoFactory = new AutoFactory(() -> Dashboard.getInstance().getAuto());

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

    NamedCommands.registerCommand("Intake", new IntakeCommand(intake, indexer));
    NamedCommands.registerCommand("Outtake", new OuttakeCommand(intake));
    NamedCommands.registerCommand("Default Superstructure", new InstantCommand(() ->superstructure.setState(SuperstructureState.DEFAULT)));
    NamedCommands.registerCommand("Speaker Idle Superstructure", new InstantCommand(() ->superstructure.setState(SuperstructureState.SPEAKER_IDLE)));
    NamedCommands.registerCommand("Speaker Score Superstructure", new InstantCommand(() ->superstructure.setState(SuperstructureState.SPEAKER_SCORE)));
    NamedCommands.registerCommand("Amp Idle Superstructure", new InstantCommand(() ->superstructure.setState(SuperstructureState.AMP_IDLE)));
    NamedCommands.registerCommand("Amp Score Superstructure", new InstantCommand(() ->superstructure.setState(SuperstructureState.AMP_SCORE)));

    //advantageScope.startRecording();

    configureBindings();
  }

  private void configureBindings() {

    /*
     * 
     */
    JoystickButton motionAimButton = new JoystickButton(rotationJoystick, 0);
    // JoystickButton orbitNoteButton = new JoystickButton(translationJoystick, 0);

    motionAimButton.whileTrue(new DriveWhileAimingCommand(
      () -> translationJoystick.getX(), 
      () -> translationJoystick.getY(), 
      fieldCentricSupplier, 
      drivetrain
    ));

    // orbitNoteButton.whileTrue(new DriveWhileOrbitingNoteCommand(
    //   () -> translationJoystick.getX(), 
    //   () -> translationJoystick.getY(), 
    //   fieldCentricSupplier, 
    //   drivetrain, vision
    // ));

    // JoystickButton raiseClimberButton = new JoystickButton(controlPanel, 12);
    // JoystickButton lowerClimberButton = new JoystickButton(controlPanel, 11);
    
    // raiseClimberButton.whileTrue(new ClimerExtendCommand(climber));
    // lowerClimberButton.whileTrue(new ClimberRetractCommand(climber));
    /*
     * Intake Button Bindings
     */
    JoystickButton intakeInButton = new JoystickButton(controlPanel, 10);
    JoystickButton intakeOutButton = new JoystickButton(controlPanel, 9);
    
    intakeInButton.whileTrue(new IntakeCommand(intake, indexer));
    intakeOutButton.whileTrue(new OuttakeCommand(intake));

    // JoystickButton toggleMusicPlayerButton = new JoystickButton(controlPanel, 2);
    // toggleMusicPlayerButton.onTrue(toggleMusic());

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
    shamperManualUpButton.whileTrue(new ShamperManualUpCommand(shamper));
    shamperManualDownButton.whileTrue(new ShamperManualDownCommand(shamper));
  }

  // public Command toggleMusic() {
  //   musicOn = !musicOn;
  //   robotState.setMusicEnableStatus(musicOn);;
  //   if (!musicOn) {new PauseMusicPlayerCommand(musicPlayer);}
  //   return null;
  // }

  public void forceRecompile() {
    autoFactory.recompile();
  }


  public void precompileAuto() {
      if (autoFactory.recompileNeeded()) {
          autoFactory.recompile();
      }
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoFactory.getCompiledAuto();
  }
}
