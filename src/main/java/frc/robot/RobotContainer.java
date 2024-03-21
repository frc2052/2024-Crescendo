// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.AutoFactory;
import frc.robot.commands.climb.ClimberRetractCommand;
import frc.robot.commands.climb.ClimberSlowRetractCommand;
import frc.robot.commands.auto.commands.drive.AimToSpeakerCommand;
import frc.robot.commands.auto.commands.shoot.ShootAutoLowCommand;
import frc.robot.commands.auto.commands.shoot.ShootCommandAuto;
import frc.robot.commands.auto.commands.shoot.ShootSubCommandAuto;
import frc.robot.commands.climb.ClimberExtendCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveWhileAimingCommand;
import frc.robot.commands.indexer.IndexerBackupCommand;
import frc.robot.commands.indexer.IndexerIndexCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeThenBackupCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.shamper.ShamperAmpCommand;
import frc.robot.commands.shamper.ShamperAngleCommand;
import frc.robot.commands.shamper.ShamperDefaultCommand;
import frc.robot.commands.shamper.ShamperManualShootCommand;
import frc.robot.commands.shamper.ShamperPivotManualDownCommand;
import frc.robot.commands.shamper.ShamperPivotManualUpCommand;
import frc.robot.commands.shamper.ShamperSubCommand;
import frc.robot.commands.shamper.ShamperTrapCommand;
import frc.robot.commands.shamper.ShamperWindDownCommand;
import frc.robot.commands.shamper.lookup.ShamperAutoAngleCommand;
import frc.robot.commands.shamper.lookup.ShamperAutoShootCommand;
import frc.robot.commands.trap.TrapReleaseCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.TrapArmSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.io.Dashboard;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
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
  private final LedSubsystem ledSubsystem;
  // private final MusicPlayerSubsystem musicPlayer;
  // private final VisionSubsystem vision;
  private final AdvantageScopeSubsystem advantageScope;
  private final TrapArmSubsystem trapArm;

  private final AutoFactory autoFactory;

  private final Joystick translationJoystick;
  private final Joystick rotationJoystick;
  private final Joystick controlPanel;

  public static boolean musicOn;
  // public RobotStatusCommunicator robotStatusCommunicator;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain = new DrivetrainSubsystem();
    intake = new IntakeSubsystem();
    indexer = new IndexerSubsystem();
    shamper = new ShamperSubsystem();
    climber = new ClimberSubsystem();
    aprilTag = AprilTagSubsystem.getInstance();
    ledSubsystem = LedSubsystem.getInstance();
    // musicPlayer = new MusicPlayerSubsystem();
    // vision = new VisionSubsystem();
    trapArm = new TrapArmSubsystem();
    advantageScope = new AdvantageScopeSubsystem(intake, shamper, climber, drivetrain, indexer);


    // robotStatusCommunicator = new RobotStatusCommunicator(musicPlayer);

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
          () -> true,
          //Dashboard.getInstance()::isFieldCentric,
          drivetrain
      )
    );

    shamper.setDefaultCommand(new ShamperDefaultCommand(shamper));

    NamedCommands.registerCommand("Robot Angle Align", new AimToSpeakerCommand(drivetrain));
    NamedCommands.registerCommand("Shoot Command", new ShootCommandAuto(shamper, indexer));
    NamedCommands.registerCommand("Sub Shoot Command", new ShootSubCommandAuto(shamper, indexer));
    NamedCommands.registerCommand("Indexer Backup", new IndexerBackupCommand(indexer));
    NamedCommands.registerCommand("Manual Angle", new ShamperAngleCommand(shamper, Constants.Shamper.Angle.SUB));
    NamedCommands.registerCommand("Manual Shoot", new ShamperManualShootCommand(shamper, ShamperSpeed.SPEAKER_IDLE));
    NamedCommands.registerCommand("Manual Index", new IndexerIndexCommand(indexer));
    NamedCommands.registerCommand("ShootAutoLow", new ShootAutoLowCommand(shamper, indexer));

    NamedCommands.registerCommand("Intake", new IntakeCommand(intake, indexer, shamper));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    /*
     * Drive Commands
     */

    JoystickButton zeroGyroButton = new JoystickButton(translationJoystick, 9);
    zeroGyroButton.onTrue(new InstantCommand(() -> {drivetrain.zeroGyro(); RobotState.getInstance().clearNavXOffset();}));

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
    JoystickButton resetIsClimbingButton = new JoystickButton(rotationJoystick, 6);

    climberRetractButton.whileTrue(new ClimberRetractCommand(climber));
    climberExtendButton.whileTrue(new ClimberExtendCommand(climber));
    climberManualRetractSlow.whileTrue(new ClimberSlowRetractCommand(climber));
    resetIsClimbingButton.whileTrue(new InstantCommand(() -> robotState.updateIsClimbing(false)));

    /*
     * Intake Button Bindings
     */

    JoystickButton intakeInButton = new JoystickButton(translationJoystick, 1);
    JoystickButton outtakeButton = new JoystickButton(translationJoystick, 3);
    
    intakeInButton.whileTrue(new IntakeThenBackupCommand(intake, indexer, shamper));
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
    Trigger shamperIdleToggleButton = new Trigger(() -> controlPanel.getY() > 0.5);
    JoystickButton shamperSubButton = new JoystickButton(translationJoystick, 2);

    shamperSubButton.whileTrue(new ShamperSubCommand(shamper, indexer));
    shamperShootButton.whileTrue(new ShamperAutoShootCommand(shamper, indexer));
    shamperWindDownButton.whileTrue(new ShamperWindDownCommand(shamper));
    shamperAmpShootButton.whileTrue(new ShamperAmpCommand(shamper, indexer));
    shamperManualShootButton.whileTrue(new ShamperManualShootCommand(shamper, ShamperSpeed.SPEAKER_SCORE));
    shamperTrapShootButton.whileTrue(new ShamperTrapCommand(shamper));
    shamperIdleToggleButton.onTrue(new InstantCommand(() -> shamper.toggleCurrentIdle()));

    /*
     *  Shamper Angle Button Bindings
     */

     JoystickButton shamper90Button = new JoystickButton(controlPanel, 8);
     Trigger shamperPodiumButton = new Trigger(() -> controlPanel.getY() < -0.5);
     JoystickButton shamperPodiumDriverButton = new JoystickButton(rotationJoystick, 4);
     JoystickButton shamperAutoAngleButton = new JoystickButton(rotationJoystick, 3);
     Trigger shamperSubwooferButton = new Trigger(() -> controlPanel.getX() > 0.5);
     Trigger shamperAmpButton = new Trigger(() -> controlPanel.getX() < -0.5);
     JoystickButton shamperClimbHeightButton = new JoystickButton(controlPanel, 9);
     JoystickButton shamperManualUpButton = new JoystickButton(controlPanel, 7);
     JoystickButton shamperManualDownButton = new JoystickButton(controlPanel, 5);

     shamper90Button.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.TRAP));
     shamperPodiumButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.PODIUM));
     shamperPodiumDriverButton.onTrue(new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT));
     shamperAutoAngleButton.onTrue(new ShamperAutoAngleCommand(shamper, indexer));
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

  // TODO: check if this before starting works for reset gyro
  public Command getAutonomousCommand() {
    return autoFactory.getCompiledAuto().beforeStarting(new InstantCommand(() -> resetGyro()));
  }

  public void resetGyro(){
    
    if (RobotState.getInstance().gyroResetNeeded()){
      drivetrain.zeroGyro();
    }
  }
}
