// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.shamper.lookup.ShamperShootCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.AimingCalculator;

public class ShamperLobOrShootCommand extends ShamperShootCommand {

  private final double blueIndexLine;
  private final double redIndexLine;
  /** Creates a new ShamperLobShotCommand. */
  public ShamperLobOrShootCommand(ShamperSubsystem shamper,IndexerSubsystem indexer) {
    super(shamper, indexer);
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(indexer, shamper);

    blueIndexLine = Constants.FieldAndRobot.BLUE_LOB_LINE;
    redIndexLine = Constants.FieldAndRobot.RED_LOB_LINE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(AimingCalculator.calculateDistanceToSpeaker(RobotState.getInstance().getRobotPose()) > 6) {
      System.out.println("LOB");
      double speed = Math.sqrt(
      Math.pow(RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond, 2) +
      Math.pow(RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond, 2)
      );

      System.out.println(speed);

      double angle = 45 + (speed * Constants.FieldAndRobot.FEED_WHILE_MOVING_ANGLE_MULTIPLIER);

      shamper.setAngle(angle);
      double upperSpeed;
      double lowerSpeed;
      if (speed > 2.5) {
        upperSpeed = ShamperSpeed.LOB.getUpper() - (ShamperSpeed.LOB.getUpper() * Constants.FieldAndRobot.LOB_SHOOTING_SPEED_MULTIPLIER);
        lowerSpeed = ShamperSpeed.LOB.getLower() - (ShamperSpeed.LOB.getLower() * Constants.FieldAndRobot.LOB_SHOOTING_SPEED_MULTIPLIER);
        shamper.setShootSpeed(
          ShamperSpeed.LOB.getUpper() - (ShamperSpeed.LOB.getUpper() * Constants.FieldAndRobot.LOB_SHOOTING_SPEED_MULTIPLIER),
          ShamperSpeed.LOB.getLower() - (ShamperSpeed.LOB.getLower() * Constants.FieldAndRobot.LOB_SHOOTING_SPEED_MULTIPLIER)
        );
      } else {
        upperSpeed = ShamperSpeed.LOB.getUpper();
        lowerSpeed = ShamperSpeed.LOB.getLower();
        shamper.setShootSpeed(ShamperSpeed.LOB);
      }
      
      double poseX = RobotState.getInstance().getRobotPose().getX();

      if (RobotState.getInstance().isRedAlliance()){
        if (poseX > redIndexLine && shamper.shooterAtSpeed(lowerSpeed, upperSpeed)) {
          indexer.indexAll();
        }
      } else {
        if (poseX < blueIndexLine && shamper.shooterAtSpeed(lowerSpeed, upperSpeed)) {
          indexer.indexAll();
        }
      }
    } else {
      System.out.println("SPEAKER");
      super.execute();
    }
  }
}
