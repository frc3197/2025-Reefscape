// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algae extends SubsystemBase {

  private final TalonFX deployMotor;
  private final TalonFX spinMotor;

  private final TalonFX leftGrabberMotor;
  private final TalonFX rightGrabberMotor;

  public Algae() {

    deployMotor = new TalonFX(Constants.AlgaeConstants.deployMotorId);
    spinMotor = new TalonFX(Constants.AlgaeConstants.spinMotorId);

    deployMotor.getConfigurator().apply(Constants.AlgaeConstants.deployMotorConfig);
    spinMotor.getConfigurator().apply(Constants.AlgaeConstants.spinMotorConfig);

    leftGrabberMotor = new TalonFX(Constants.AlgaeConstants.leftGrabberMotorId);
    rightGrabberMotor = new TalonFX(Constants.AlgaeConstants.rightGrabberMotorId);

    leftGrabberMotor.getConfigurator().apply(Constants.AlgaeConstants.leftGrabberMotorConfig);
    rightGrabberMotor.getConfigurator().apply(Constants.AlgaeConstants.rightGrabberMotorConfig);

  }

  public Command setAlgaeGrabbers(double speed) {
    return Commands.runOnce(() -> {
      leftGrabberMotor.set(speed);
      rightGrabberMotor.set(speed);
    }, this);
  }
  
}
