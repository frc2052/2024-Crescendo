// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;

/*
 *  This command ends when we are past the centerline to cancel our auto centerline pickup command
 */
public class AutoPastHotdogCenterlineCommand extends Command {
  private RobotState robotState;
  private double robotY;
  private boolean pastCenter;
  /** Creates a new AutoPastCenterlineCommand. */
  public AutoPastHotdogCenterlineCommand() {
    this.robotState = RobotState.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override 
  public void execute() {
    robotY = robotState.getRobotPose().getX();

    System.out.println("robot y " + robotY + " midline: " + Constants.FieldAndRobot.CENTER_LINE_HOTDOG);
    if(!robotState.isRedAlliance() && robotY < Constants.FieldAndRobot.CENTER_LINE_HOTDOG) {
      System.out.println("PASSED HOTDOG CENTER");
      pastCenter = true;
    } else if (robotState.isRedAlliance() && robotY > Constants.FieldAndRobot.CENTER_LINE_HOTDOG){
      pastCenter = true;
    } else {
      pastCenter = false;
    }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pastCenter;
  }
}
