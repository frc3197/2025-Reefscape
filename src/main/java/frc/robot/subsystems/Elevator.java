// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final Encoder elevatorEncoder;

  public Elevator() {
    leftMotor = new TalonFX(Constants.ElevatorConstants.leftElevatorId);
    rightMotor = new TalonFX(Constants.ElevatorConstants.rightElevatorId);

    leftMotor.getConfigurator().apply(Constants.ElevatorConstants.leftElevatorConfig);
    rightMotor.getConfigurator().apply(Constants.ElevatorConstants.rightElevatorConfig);

    elevatorEncoder = new Encoder(Constants.ElevatorConstants.elevatorEncoderChannelA, Constants.ElevatorConstants.elevatorEncoderChannelB);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Elevator encoder", readEncoder());
    SmartDashboard.putNumber("Elevator height", getElevatorHeight());

  }

  public InstantCommand getManualCommand(double speed) {
    return new InstantCommand(() -> {
      leftMotor.set(speed);
      rightMotor.set(speed);
    });
  }

  public double readEncoder() {
    return elevatorEncoder.get();
  }

  private void resetEncoder() {
    elevatorEncoder.reset();
  }

  public InstantCommand getResetCommand() {
    return new InstantCommand(this::resetEncoder);
  }

  public double getElevatorHeight() {
    return (readEncoder() * 10) + 5;
  }
}
