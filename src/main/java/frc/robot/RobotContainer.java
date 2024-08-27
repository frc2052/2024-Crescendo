// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.AutoFactory;
import frc.robot.commands.climb.ClimberRetractCommand;
import frc.robot.commands.climb.ClimberSlowRetractCommand;
import frc.robot.commands.autonomous.drive.AimToSpeakerCommand;
import frc.robot.commands.autonomous.drive.AutoCenterLinePickupCommand;
import frc.robot.commands.autonomous.intake.AutoIntakeCommand;
import frc.robot.commands.autonomous.shamper.PreShootCommandAuto;
import frc.robot.commands.autonomous.shamper.ShootCommandAuto;
import frc.robot.commands.autonomous.shamper.ShootSubCommandAuto;
import frc.robot.commands.climb.ClimberExtendCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveWhileAimToAngle;
import frc.robot.commands.drive.DriveWhileAimingCommand;
import frc.robot.commands.drive.DriveWhileGamePieceAlign;
import frc.robot.commands.drive.DriveWhileLobbingCommand;
import frc.robot.commands.drive.GamePieceAlignmentCommand;
import frc.robot.commands.indexer.IndexerIndexCommand;
import frc.robot.commands.intake.IntakeThenBackupCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.music.PlayFOTBCommand;
import frc.robot.commands.shamper.ShamperAmpCommand;
import frc.robot.commands.shamper.ShamperDefaultCommand;
import frc.robot.commands.shamper.ShamperLobOrShootCommand;
import frc.robot.commands.shamper.lookup.ShamperAimAngleCommand;
import frc.robot.commands.shamper.pivot.ShamperAngleCommand;
import frc.robot.commands.shamper.pivot.ShamperPivotManualUpCommand;
import frc.robot.commands.shamper.shoot.ShamperManualShootCommand;
import frc.robot.commands.shamper.shoot.ShamperTrapCommand;
import frc.robot.commands.trap.TrapToggleCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.MusicPlayerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.TrapArmSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.RobotStatusCommunicator;
import frc.robot.util.io.Dashboard;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
  private final LedSubsystem ledSubsystem;
  private final ForwardPixySubsystem pixy;
  private final MusicPlayerSubsystem musicPlayer;
  // private final VisionSubsystem vision;
  private final AdvantageScopeSubsystem advantageScope;
  private final TrapArmSubsystem trapArm;

  private final AutoFactory autoFactory;

  private final Joystick translationJoystick;
  private final Joystick rotationJoystick;
  private final Joystick controlPanel;

  public static boolean musicOn;
  public RobotStatusCommunicator robotStatusCommunicator;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain = new DrivetrainSubsystem();
    intake = new IntakeSubsystem();
    indexer = new IndexerSubsystem();
    shamper = new ShamperSubsystem();
    climber = new ClimberSubsystem();
    aprilTag = AprilTagSubsystem.getInstance();
    ledSubsystem = LedSubsystem.getInstance();
    musicPlayer = new MusicPlayerSubsystem();
    // vision = new VisionSubsystem();
    trapArm = new TrapArmSubsystem();
    advantageScope = new AdvantageScopeSubsystem(intake, shamper, climber, drivetrain, indexer);
    pixy = new ForwardPixySubsystem();


    robotStatusCommunicator = new RobotStatusCommunicator(musicPlayer);

    musicOn = true;
    ledSubsystem.enableLEDs();

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
          Dashboard.getInstance()::isFieldCentric,
          //Dashboard.getInstance()::isFieldCentric,
          drivetrain
      )
    );

    shamper.setDefaultCommand(new ShamperDefaultCommand(shamper));

    NamedCommands.registerCommand("Shoot Command", new ShootCommandAuto(shamper, indexer));
    NamedCommands.registerCommand("Sub Shoot Command", new ShootSubCommandAuto(shamper, indexer));
    NamedCommands.registerCommand("Intake Command", new AutoIntakeCommand(intake, indexer));
    NamedCommands.registerCommand("Aim Speaker Command", new AimToSpeakerCommand(drivetrain).withTimeout(.75));
    NamedCommands.registerCommand("Pre-Shoot Command", new PreShootCommandAuto(shamper));
    NamedCommands.registerCommand("Note Alignment Command", new GamePieceAlignmentCommand(2, -.6, -.4, drivetrain, pixy));
      
    Rotation2d towardsAmpSide = Rotation2d.fromDegrees(RobotState.getInstance().isRedAlliance() ? 270 : 90);
    Rotation2d towardsSourceSide = Rotation2d.fromDegrees(RobotState.getInstance().isRedAlliance() ? 90 : 270);

    // NamedCommands.registerCommand("Game Piece Alignment Towards Amp", new AutoCenterLineNotePickupCommand(2.0, -.6, -.4, () -> towardsAmpSide, drivetrain, pixy, intake));
    // NamedCommands.registerCommand("Game Piece Alignment Towards Source", new AutoCenterLineNotePickupCommand(2, -.6, -.4, () -> towardsSourceSide, drivetrain, pixy, intake));
    NamedCommands.registerCommand("AMP SIDE Game Piece Alignment", new AutoCenterLinePickupCommand(drivetrain, pixy, intake, indexer, shamper, towardsAmpSide));
    NamedCommands.registerCommand("SOURCE SIDE Game Piece Alignment", new AutoCenterLinePickupCommand(drivetrain, pixy, intake, indexer, shamper, towardsSourceSide));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // JoystickButton customShotButton = new JoystickButton(rotationJoystick, 11);
    // customShotButton.whileTrue(new ShamperCustomShotCommand(shamper, indexer));
    // JoystickButton customAngleButton = new JoystickButton(rotationJoystick, 10);
    // customAngleButton.whileTrue(new ShamperCustomAngle(shamper));

    JoystickButton gamePieceDriveButton = new JoystickButton(translationJoystick, 5);
    gamePieceDriveButton.whileTrue(new DriveWhileGamePieceAlign(
      () -> translationJoystick.getY(), 
      () -> translationJoystick.getX(),
      () -> rotationJoystick.getX(), 
      0.5, 
      1,
      drivetrain, 
      pixy)
    );

    ParallelDeadlineGroup specialIntakeCommand = new ParallelDeadlineGroup(
      new AutoIntakeCommand(intake, indexer), 
      new GamePieceAlignmentCommand(2, -.4, .3, drivetrain, pixy)
    );

    JoystickButton gamePieceAlignmentButton = new JoystickButton(translationJoystick, 7);
    gamePieceAlignmentButton.whileTrue(specialIntakeCommand);

    /*
     * Drive Button Bindings
     */

    JoystickButton zeroGyroButton = new JoystickButton(translationJoystick, 9);
    zeroGyroButton.onTrue(new InstantCommand(() -> drivetrain.zeroOdometry()));

    JoystickButton driveWhileAimingButton = new JoystickButton(rotationJoystick, 2);
    driveWhileAimingButton.whileTrue(new DriveWhileAimingCommand(
      () -> translationJoystick.getY(), 
      () -> translationJoystick.getX(), 
      () -> true, 
      drivetrain
    ));

    JoystickButton aimToAmpButton = new JoystickButton(rotationJoystick, 4);
    Rotation2d ampDirection = Rotation2d.fromDegrees(RobotState.getInstance().isRedAlliance() ? 90 : 270);
    aimToAmpButton.whileTrue(new DriveWhileAimToAngle(
      () -> translationJoystick.getY(), 
      () -> translationJoystick.getX(),  
      () -> ampDirection,
      Dashboard.getInstance()::isFieldCentric,
      drivetrain
    ));

    // JoystickButton aimLobButton = new JoystickButton(rotationJoystick, 5);
    // aimLobButton.whileTrue(new ShamperLobCommand(shamper, indexer));
    JoystickButton autoLobButton = new JoystickButton(rotationJoystick, 5);
    autoLobButton.onTrue(new InstantCommand(() -> robotState.setIsLobbing(true))).onFalse(new InstantCommand(() -> robotState.setIsLobbing(false)));

    autoLobButton.whileTrue(new DriveWhileLobbingCommand(
      () -> translationJoystick.getY(), 
      () -> translationJoystick.getX(), 
      () -> true, 
      drivetrain));

    /*
     *  Climber Button Bindings
     */
    JoystickButton climberRetractButton = new JoystickButton(controlPanel, 6);
    JoystickButton climberExtendButton = new JoystickButton(controlPanel, 1);
    JoystickButton climberManualRetractSlow = new JoystickButton(controlPanel, 4);
    JoystickButton resetIsClimbingButton = new JoystickButton(rotationJoystick, 6);

    climberRetractButton.whileTrue(new ClimberRetractCommand(climber));
    climberExtendButton.whileTrue(new ClimberExtendCommand(climber));
    climberManualRetractSlow.whileTrue(new ClimberSlowRetractCommand(climber));
    resetIsClimbingButton.whileTrue(new InstantCommand(() -> robotState.updateIsClimbing(false)));

    /*
     * Intake Button Bindings
     */

    JoystickButton intakeInButton = new JoystickButton(translationJoystick, 1);
    JoystickButton intakeOverrideButton = new JoystickButton(translationJoystick, 4);
    JoystickButton outtakeButton = new JoystickButton(translationJoystick, 3);
    intakeInButton.whileTrue(new IntakeThenBackupCommand(intake, indexer, shamper));
    intakeOverrideButton.onTrue(new InstantCommand(() -> robotState.updateNoteDetectorOverride(true))).onFalse(new InstantCommand(() -> robotState.updateNoteDetectorOverride(false)));
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
    JoystickButton shamperAmpShootButton = new JoystickButton(controlPanel, 11);
    JoystickButton shamperManualShootButton = new JoystickButton(controlPanel, 12);
    JoystickButton shamperTrapShootButton = new JoystickButton(controlPanel, 2);
    Trigger shamperIdleToggleButton = new Trigger(() -> controlPanel.getY() > 0.5);
    // JoystickButton shamperCustomAngleButton = new JoystickButton(translationJoystick, 7);
    // JoystickButton shamperSubButton = new JoystickButton(translationJoystick, 2);

    shamperShootButton.whileTrue(new ShamperLobOrShootCommand(shamper, indexer));
    shamperAmpShootButton.whileTrue(new ShamperAmpCommand(shamper, indexer));
    shamperManualShootButton.whileTrue(new ShamperManualShootCommand(shamper, ShamperSpeed.SPEAKER_SCORE));
    shamperTrapShootButton.whileTrue(new ShamperTrapCommand(shamper, indexer, trapArm));
    shamperIdleToggleButton.onTrue(new InstantCommand(() -> shamper.toggleCurrentIdle()));
    // shamperCustomAngleButton.onTrue(new ShamperCustomAngle(shamper));
    // shamperSubButton.whileTrue(new ShamperSubCommand(shamper, indexer));

    /*
     *  Shamper Angle Button Bindings
     */

     JoystickButton shamper90Button = new JoystickButton(controlPanel, 8);
     Trigger shamperPodiumButton = new Trigger(() -> controlPanel.getY() < -0.5);
     JoystickButton shamperAutoAngleButton = new JoystickButton(rotationJoystick, 3);
     Trigger shamperSubwooferButton = new Trigger(() -> controlPanel.getX() > 0.5);
     Trigger shamperAmpButton = new Trigger(() -> controlPanel.getX() < -0.5);
     JoystickButton shamperClimbHeightButton = new JoystickButton(controlPanel, 9);
     JoystickButton shamperTrapButton = new JoystickButton(controlPanel, 5);
     JoystickButton shamperManualUpButton = new JoystickButton(controlPanel, 7);
    //  JoystickButton shamperManualDownButton = new JoystickButton(controlPanel, 5);

     shamper90Button.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.NINETY));
     shamperPodiumButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.PODIUM));
     shamperAutoAngleButton.whileTrue(new ShamperAimAngleCommand(shamper));
     shamperSubwooferButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.SUB));
     shamperAmpButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.AMP));
     shamperClimbHeightButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.CLIMB));
     shamperTrapButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.TRAP));
     shamperManualUpButton.whileTrue(new ShamperPivotManualUpCommand(shamper));
    //  shamperManualDownButton.whileTrue(new ShamperPivotManualDownCommand(shamper));

    /*
     * Trap Button Bindings
     */

    JoystickButton trapReleaseButton = new JoystickButton(controlPanel, 3);

    trapReleaseButton.whileTrue(new TrapToggleCommand(trapArm));
    
    /*
     * Music Player Toggle
     */
    // JoystickButton toggleMusicPlayerButton = new JoystickButton(controlPanel, 2);
    // toggleMusicPlayerButton.onTrue(toggleMusic());

    JoystickButton musictest = new JoystickButton(translationJoystick, 10);
    musictest.onTrue(new PlayFOTBCommand(musicPlayer));
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
    return autoFactory.getCompiledAuto();
  }

  public void resetGyro(){
    // if (RobotState.getInstance().gyroResetNeeded()){
    //   drivetrain.zeroOdometry();
    // }
  }
}
