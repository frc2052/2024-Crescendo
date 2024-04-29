// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveWhileAimAmpCommand extends DriveCommand {
    private final PIDController rotationController;
    private final Supplier<Rotation2d> targetRotation;

  /** Creates a new DriveWhileAimAmpCommand. */
  public DriveWhileAimAmpCommand(        
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        Supplier<Rotation2d> targetRotation,
        BooleanSupplier fieldCentricSupplier,
        DrivetrainSubsystem drivetrain
    ) {
    super(xSupplier, ySupplier, () -> 0, fieldCentricSupplier, drivetrain);

        rotationController = new PIDController(2.5, 0, 0.1);
        rotationController.enableContinuousInput(0, 360);
        rotationController.setTolerance(2);

        this.targetRotation = targetRotation;
  }

  @Override
  public void initialize() {
      // Normalize the angle value between [0, 360]
      rotationController.setSetpoint(targetRotation.get().getDegrees());
  }

  // Called when the command is initially scheduled.
  @Override
  protected double getRotation() {
    double gyroDegrees = RobotState.getInstance().getRotation2d360().getDegrees();

    // Calculate PID value along with feedforward constant to assist with minor adjustments.
    double rotationValue = rotationController.calculate(gyroDegrees) / 360;
    if (!rotationController.atSetpoint()) {
        return rotationValue + Math.copySign(0.025, rotationValue);
    } else {
        return 0;
    }
  }
}
