// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class AutoAimDownCenterlineCommand extends Command {
  private RobotState robotState;

  private PIDController rotationController;


  /** Creates a new AutoAimDownCenterlineCommand. */
  public AutoAimDownCenterlineCommand() {   
    
    robotState = RobotState.getInstance();

    rotationController = new PIDController(2.5, 0, 0.1);
    rotationController.enableContinuousInput(0, 360);
    rotationController.setTolerance(2);
    


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    double gyroDegrees = robotState.getRotation2d360().getDegrees();

     // Calculate PID value along with feedforward constant to assist with minor adjustments.
     double rotationValue = rotationController.calculate(gyroDegrees) / 360;

     if (!rotationController.atSetpoint()) {
         rotationValue += Math.copySign(0.025, rotationValue);
     }
 
     rotationController = new PIDController(2.5, 0, 0.1);
     rotationController.enableContinuousInput(0, 360);
     rotationController.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
