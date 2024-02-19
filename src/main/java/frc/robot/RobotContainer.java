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
//import frc.robot.commands.music.PauseMusicPlayerCommand;
//import frc.robot.commands.music.PlayActivationJingleCommand;
import frc.robot.commands.shamper.ShamperManualShootCommand;
import frc.robot.commands.shamper.ShamperPivotManualDownCommand;
import frc.robot.commands.shamper.ShamperPivotManualUpCommand;
import frc.robot.commands.shamper.ShamperShootCommand;
import frc.robot.commands.shamper.ShamperStopCommand;
//import frc.robot.subsystems.AdvantageScopeSubsystem;
//import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
//import frc.robot.subsystems.MusicPlayerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.Superstructure;
//import frc.robot.subsystems.TrapArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.subsystems.Superstructure.SuperstructureState;
//import frc.robot.util.RobotStatusCommunicator;
import frc.robot.util.io.Dashboard;

import java.util.function.BooleanSupplier;

import javax.swing.JOptionPane;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  // private final Superstructure superstructure;

  private final AutoFactory autoFactory;

  private final Joystick translationJoystick;
  private final Joystick rotationJoystick;
  private final Joystick controlPanel;

  public static boolean musicOn;
  //public RobotStatusCommunicator robotStatusCommunicator;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain = new DrivetrainSubsystem();
    intake = new IntakeSubsystem();
    indexer = new IndexerSubsystem();
    shamper = new ShamperSubsystem();
    //climber = new ClimberSubsystem();
    // musicPlayer = new MusicPlayerSubsystem();
    // vision = new VisionSubsystem();
    // trapArm = new TrapArmSubsystem();
    // advantageScope = new AdvantageScopeSubsystem(intake, shamper, climber, drivetrain, musicPlayer, vision, indexer, trapArm);

    //superstructure = new Superstructure(shamper, climber, indexer);
    // superstructure = new Superstructure(shamper, indexer);

    // robotStatusCommunicator = new RobotStatusCommunicator(musicPlayer);

    musicOn = true;

    autoFactory = new AutoFactory(() -> Dashboard.getInstance().getAuto());

    translationJoystick = new Joystick(0);
    rotationJoystick = new Joystick(1);
    controlPanel = new Joystick(2);

    drivetrain.setDefaultCommand(
      new DriveCommand(
          // Forward velocity supplier.
          translationJoystick::getY,
          // Sideways velocity supplier.
          translationJoystick::getX,
          // Rotation velocity supplier.
          rotationJoystick::getX,
          () ->false,
          //Dashboard.getInstance()::isFieldCentric,
          drivetrain
      )
    );

    // NamedCommands.registerCommand("Intake", new IntakeCommand(intake, indexer));
    // NamedCommands.registerCommand("Outtake", new OuttakeCommand(intake, indexer));
    // NamedCommands.registerCommand("Default Superstructure", new InstantCommand(() ->superstructure.setState(SuperstructureState.DEFAULT)));
    // NamedCommands.registerCommand("Speaker Idle Superstructure", new InstantCommand(() ->superstructure.setState(SuperstructureState.SPEAKER_IDLE)));
    // NamedCommands.registerCommand("Speaker Score Superstructure", new InstantCommand(() ->superstructure.setState(SuperstructureState.SPEAKER_SCORE)));
    // NamedCommands.registerCommand("Amp Idle Superstructure", new InstantCommand(() ->superstructure.setState(SuperstructureState.AMP_IDLE)));
    // NamedCommands.registerCommand("Amp Score Superstructure", new InstantCommand(() ->superstructure.setState(SuperstructureState.AMP_SCORE)));

    //advantageScope.startRecording();

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    /*
     * Drive Commands
     */
    // JoystickButton driveWhileAimingButton = new JoystickButton(rotationJoystick, 1);
    // JoystickButton orbitNoteButton = new JoystickButton(translationJoystick, 0);

    // driveWhileAimingButton.whileTrue(new DriveWhileAimingCommand(
    //   () -> translationJoystick.getX(), 
    //   () -> translationJoystick.getY(), 
    //   fieldCentricSupplier, 
    //   drivetrain
    // ));

    // orbitNoteButton.whileTrue(new DriveWhileOrbitingNoteCommand(
    //   () -> translationJoystick.getX(), 
    //   () -> translationJoystick.getY(), 
    //   fieldCentricSupplier, 
    //   drivetrain, vision
    // ));


    /*
     *  Climber Button Bindings
     */
    // JoystickButton raiseClimberButton = new JoystickButton(controlPanel, 12);
    // JoystickButton lowerClimberButton = new JoystickButton(controlPanel, 11);
    
    // raiseClimberButton.whileTrue(new ClimberExtendCommand(climber));
    // lowerClimberButton.whileTrue(new ClimberRetractCommand(climber));
    /*
     * Intake Button Bindings
     */
    JoystickButton intakeInButton = new JoystickButton(translationJoystick, 1);
    JoystickButton intakeOutButton = new JoystickButton(translationJoystick, 11);
    
    intakeInButton.whileTrue(new IntakeCommand(intake, indexer));
    intakeOutButton.whileTrue(new OuttakeCommand(intake, indexer));

    /*
     *  Manual Shamper Button Bindings
     */
    // JoystickButton shamperManualUpButton = new JoystickButton(controlPanel, 3);
    // JoystickButton shamperManualDownButton = new JoystickButton(controlPanel, 4);
    JoystickButton shamperManualShotButton = new JoystickButton(rotationJoystick, 1);
    JoystickButton indexPlsBUtton = new JoystickButton(rotationJoystick, 2);

    indexPlsBUtton.onTrue(new InstantCommand(() -> indexer.indexAll()));
    
    // shamperManualUpButton.whileTrue(new ShamperPivotManualUpCommand(shamper));
    // shamperManualDownButton.whileTrue(new ShamperPivotManualDownCommand(shamper));
    shamperManualShotButton.whileTrue(new ShamperShootCommand(shamper, indexer, ShamperSpeed.SPEAKER_SCORE)).onFalse(new ShamperStopCommand(shamper));
    /*
     *  Superstructure Position Button Bindings
     */

    // JoystickButton shamperShootButton = new JoystickButton(rotationJoystick, 1);
    // JoystickButton shamperAmpIdleButton = new JoystickButton(rotationJoystick, 2);
    // JoystickButton shamperSpeakerIdleAimButon = new JoystickButton(rotationJoystick, 3);
    // JoystickButton shamperPodiumIdleButton = new JoystickButton(rotationJoystick, 4);
    // JoystickButton shamperDefaultButton = new JoystickButton(rotationJoystick, 5);

    // shamperAmpIdleButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.AMP_IDLE)));
    // shamperSpeakerIdleAimButon.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.SPEAKER_IDLE)));
    // shamperPodiumIdleButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.PODIUM_IDLE)));
    
    // shamperShootButton.onTrue(new InstantCommand(() -> {
    //   if(superstructure.getState() == SuperstructureState.AMP_IDLE) {
    //     superstructure.setState(SuperstructureState.AMP_SCORE);
    //   } else if (superstructure.getState() == SuperstructureState.SPEAKER_IDLE) {
    //     superstructure.setState(SuperstructureState.SPEAKER_SCORE);
    //   } else if (superstructure.getState() == SuperstructureState.PODIUM_IDLE) {
    //     superstructure.setState(SuperstructureState.PODIUM_SCORE);
    //   }
    // })).onFalse(new InstantCommand(() -> superstructure.setState(SuperstructureState.DEFAULT)));

    // shamperDefaultButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.DEFAULT)));
    
    /*
     * Music Player Toggle
     */
    // JoystickButton toggleMusicPlayerButton = new JoystickButton(controlPanel, 2);
    // toggleMusicPlayerButton.onTrue(toggleMusic());
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
