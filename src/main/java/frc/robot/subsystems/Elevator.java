// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  public Elevator() {
    leftMotor = new TalonFX(Constants.ElevatorConstants.leftElevatorId);
    rightMotor = new TalonFX(Constants.ElevatorConstants.rightElevatorId);

    leftMotor.getConfigurator().apply(Constants.ElevatorConstants.leftElevatorConfig);
    rightMotor.getConfigurator().apply(Constants.ElevatorConstants.rightElevatorConfig);
  }

  @Override
  public void periodic() {
  }

  public InstantCommand getManualCommand(double speed) {
    return new InstantCommand(() -> {
      leftMotor.set(speed);
      rightMotor.set(speed);
    });
  }
}
