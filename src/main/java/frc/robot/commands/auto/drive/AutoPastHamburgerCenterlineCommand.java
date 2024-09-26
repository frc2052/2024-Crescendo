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
public class AutoPastHamburgerCenterlineCommand extends Command {
  private RobotState robotState;
  boolean pastCenter;
  private double robotX;
  /** Creates a new AutoPastCenterlineCommand. */
  public AutoPastHamburgerCenterlineCommand() {
    this.robotState = RobotState.getInstance();
  }

  @Override 
  public void execute() {
    robotX = robotState.getRobotPose().getX();

    System.out.println("robot x " + robotX + " midline: " + Constants.FieldAndRobot.CENTER_LINE_HAMBURGER);
    if(!robotState.isRedAlliance() && robotX > Constants.FieldAndRobot.CENTER_LINE_HAMBURGER) {
      System.out.println("PASSED HAMBURGER CENTER");
      pastCenter = true;
    } else if (robotState.isRedAlliance() && robotX < Constants.FieldAndRobot.CENTER_LINE_HAMBURGER){
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
