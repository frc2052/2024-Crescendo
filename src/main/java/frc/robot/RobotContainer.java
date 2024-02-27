// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.AutoFactory;
import frc.robot.commands.climb.ClimberRetractCommand;
import frc.robot.commands.climb.ClimberSlowRetractCommand;
import frc.robot.commands.GyroOffsetCommand;
import frc.robot.commands.auto.drive.AimToSpeakerCommand;
import frc.robot.commands.auto.drive.BasicAuto;
import frc.robot.commands.auto.shoot.ShootCommandAuto;
import frc.robot.commands.climb.ClimberExtendCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveWhileAimingCommand;
import frc.robot.commands.indexer.IndexerIndexCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.shamper.ShamperAmpCommand;
import frc.robot.commands.shamper.ShamperAngleCommand;
import frc.robot.commands.shamper.ShamperDefaultCommand;
import frc.robot.commands.shamper.ShamperIdleCommand;
import frc.robot.commands.shamper.ShamperManualShootCommand;
import frc.robot.commands.shamper.ShamperPivotManualDownCommand;
import frc.robot.commands.shamper.ShamperPivotManualUpCommand;
//import frc.robot.commands.music.PauseMusicPlayerCommand;
//import frc.robot.commands.music.PlayActivationJingleCommand;
import frc.robot.commands.shamper.ShamperShootCommand;
import frc.robot.commands.shamper.ShamperTrapCommand;
import frc.robot.commands.shamper.ShamperWindDownCommand;
import frc.robot.commands.trap.TrapReleaseCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.AdvantageScopeSubsystem;
//import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
//import frc.robot.subsystems.MusicPlayerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.TrapArmSubsystem;
// import frc.robot.subsystems.Superstructure;
//import frc.robot.subsystems.TrapArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
// import frc.robot.subsystems.Superstructure.SuperstructureState;
//import frc.robot.util.RobotStatusCommunicator;
import frc.robot.util.io.Dashboard;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private RobotState robotState = RobotState.getInstance();
  private final DrivetrainSubsystem drivetrain;
  private final IntakeSubsystem intake;
  private final ShamperSubsystem shamper;
  private final IndexerSubsystem indexer;
  private final ClimberSubsystem climber;
  private final AprilTagSubsystem aprilTag;
  // private final MusicPlayerSubsystem musicPlayer;
  // private final VisionSubsystem vision;
  private final AdvantageScopeSubsystem advantageScope;
  private final TrapArmSubsystem trapArm;

  //private final Superstructure superstructure;

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
    climber = new ClimberSubsystem();
    aprilTag = AprilTagSubsystem.getInstance();
    // musicPlayer = new MusicPlayerSubsystem();
    // vision = new VisionSubsystem();
    trapArm = new TrapArmSubsystem();
    advantageScope = new AdvantageScopeSubsystem(intake, shamper, climber, drivetrain, indexer);

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
          () -> true,
          //Dashboard.getInstance()::isFieldCentric,
          drivetrain
      )
    );

    shamper.setDefaultCommand(new ShamperDefaultCommand(shamper));

    NamedCommands.registerCommand("Robot Angle Align", new AimToSpeakerCommand(drivetrain));
    NamedCommands.registerCommand("Adjust Angle and Score", new ShamperShootCommand(shamper, indexer));
    NamedCommands.registerCommand("Shoot Command", new ShootCommandAuto(shamper, indexer));
    NamedCommands.registerCommand("Manual Angle", new ShamperAngleCommand(shamper, Constants.Shamper.Angle.SUB));
    NamedCommands.registerCommand("Manual Shoot", new ShamperManualShootCommand(shamper, ShamperSpeed.SPEAKER_IDLE));
    NamedCommands.registerCommand("Manual Index", new IndexerIndexCommand(indexer));
    // NamedCommands.registerCommand("Gyro Offset", new GyroOffsetCommand());

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

    JoystickButton zeroGyroButton = new JoystickButton(translationJoystick, 9);
    zeroGyroButton.onTrue(new InstantCommand(() -> drivetrain.zeroGyro()));
    JoystickButton driveWhileAimingButton = new JoystickButton(rotationJoystick, 2);

    driveWhileAimingButton.whileTrue(new DriveWhileAimingCommand(
      () -> translationJoystick.getY(), 
      () -> translationJoystick.getX(), 
      () -> true, 
      drivetrain
    ));

    /*
     *  Climber Button Bindings
     */
    JoystickButton climberRetractButton = new JoystickButton(controlPanel, 6);
    JoystickButton climberExtendButton = new JoystickButton(controlPanel, 1);
    JoystickButton climberManualRetractSlow = new JoystickButton(controlPanel, 4);

    climberRetractButton.whileTrue(new ClimberRetractCommand(climber));
    climberExtendButton.whileTrue(new ClimberExtendCommand(climber));
    climberManualRetractSlow.whileTrue(new ClimberSlowRetractCommand(climber));

    /*
     * Intake Button Bindings
     */

    JoystickButton intakeInButton = new JoystickButton(translationJoystick, 1);
    JoystickButton outtakeButton = new JoystickButton(translationJoystick, 3);
    
    intakeInButton.whileTrue(new IntakeCommand(intake, indexer, shamper));
    outtakeButton.whileTrue(new OuttakeCommand(intake, indexer, shamper));

    /*
     *  Index Button Binding
     */

    JoystickButton indexManualButton = new JoystickButton(controlPanel, 10);
    indexManualButton.whileTrue(new IndexerIndexCommand(indexer));
    /*
     *  Shooter Button Bindings
     */

    JoystickButton shamperShootButton = new JoystickButton(rotationJoystick, 1);
    JoystickButton shamperWindDownButton = new JoystickButton(rotationJoystick, 5);
    JoystickButton shamperAmpShootButton = new JoystickButton(controlPanel, 11);
    JoystickButton shamperManualShootButton = new JoystickButton(controlPanel, 12);
    JoystickButton shamperTrapShootButton = new JoystickButton(controlPanel, 2);
    Trigger shamperIdleButton = new Trigger(() -> controlPanel.getY() > 0.5);

    shamperShootButton.whileTrue(new ShamperShootCommand(shamper, indexer));
    shamperWindDownButton.whileTrue(new ShamperWindDownCommand(shamper));
    shamperAmpShootButton.whileTrue(new ShamperAmpCommand(shamper));
    shamperManualShootButton.whileTrue(new ShamperManualShootCommand(shamper, ShamperSpeed.SPEAKER_SCORE));
    shamperTrapShootButton.whileTrue(new ShamperTrapCommand(shamper));
    shamperIdleButton.whileTrue(new ShamperIdleCommand(shamper));

    /*
     *  Shamper Angle Button Bindings
     */

     JoystickButton shamper90Button = new JoystickButton(controlPanel, 8);
     Trigger shamperPodiumButton = new Trigger(() -> controlPanel.getY() < -0.5);
     JoystickButton shamperPodiumDriverButton = new JoystickButton(rotationJoystick, 4);
     Trigger shamperSubwooferButton = new Trigger(() -> controlPanel.getX() > 0.5);
     Trigger shamperAmpButton = new Trigger(() -> controlPanel.getX() < -0.5);
     JoystickButton shamperClimbHeightButton = new JoystickButton(controlPanel, 9);
     JoystickButton shamperManualUpButton = new JoystickButton(controlPanel, 7);
     JoystickButton shamperManualDownButton = new JoystickButton(controlPanel, 5);

     shamper90Button.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.TRAP));
     shamperPodiumButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT));
     shamperPodiumDriverButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT));
     shamperSubwooferButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.SUB));
     shamperAmpButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.AMP));
     shamperClimbHeightButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.CLIMB));
     shamperManualUpButton.onTrue(new ShamperPivotManualUpCommand(shamper));
     shamperManualDownButton.onTrue(new ShamperPivotManualDownCommand(shamper));


    /*
     * Trap Button Bindings
     */

    JoystickButton trapReleaseButton = new JoystickButton(controlPanel, 3);

    trapReleaseButton.whileTrue(new TrapReleaseCommand(trapArm));

    /*
     *  Superstructure Position Button Bindings
     */

    // JoystickButton shamperShootButton = new JoystickButton(rotationJoystick, 1);
    // JoystickButton shamperAmpIdleButton = new JoystickButton(rotationJoystick, 4);
    // JoystickButton superstructureIntakeButton = new JoystickButton(rotationJoystick, 2);
    // // JoystickButton shamperSpeakerIdleAimButon = new JoystickButton(rotationJoystick, 3);
    // JoystickButton shamperPodiumIdleButton = new JoystickButton(rotationJoystick, 3);
    // JoystickButton shamperDefaultButton = new JoystickButton(rotationJoystick, 5);

    // shamperAmpIdleButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.AMP_IDLE)));
    // // shamperSpeakerIdleAimButon.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.SPEAKER_IDLE)));
    // superstructureIntakeButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.INTAKE)));
    // shamperPodiumIdleButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.PODIUM_IDLE)));
    // shamperDefaultButton.onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureState.DEFAULT)));
    
    // shamperShootButton.onTrue(new InstantCommand(() -> {
    //   if(superstructure.getState() == SuperstructureState.AMP_IDLE) {
    //     superstructure.setState(SuperstructureState.AMP_SCORE);
    //   // } else if (superstructure.getState() == SuperstructureState.SPEAKER_IDLE) {
    //   //   superstructure.setState(SuperstructureState.SPEAKER_SCORE);
    //   } else if (superstructure.getState() == SuperstructureState.PODIUM_IDLE) {
    //     superstructure.setState(SuperstructureState.PODIUM_SCORE);
    //   } else {
    //     System.out.println("WASN'T IDLING*********");
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
    return new PathPlannerAuto("Center 4");
    // return new BasicAuto(drivetrain, shamper, indexer);
  }

  public void resetGyro(){
    drivetrain.zeroGyro();
  }
}
