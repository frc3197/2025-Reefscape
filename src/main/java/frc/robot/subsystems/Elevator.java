// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.ErrorMode;
import frc.robot.util.ErrorBody;

public class Elevator extends SubsystemBase {

  // Elevation motors
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  // Elevation height encoder
  private final Encoder elevatorEncoder;

  // Elevator
  private ElevatorFeedforward elevatorFeedforward;
  private final PIDController testPID;

  // Target elevation height
  private double targetHeightTicks = 0;

  public Elevator() {

    this.leftMotor = new TalonFX(Constants.ElevatorConstants.leftElevatorMotorId);
    this.rightMotor = new TalonFX(Constants.ElevatorConstants.rightElevatorMotorId);

    this.leftMotor.getConfigurator().apply(Constants.ElevatorConstants.leftElevatorMotorConfig);
    this.rightMotor.getConfigurator().apply(Constants.ElevatorConstants.rightElevatorMotorConfig);

    this.leftMotor.setNeutralMode(NeutralModeValue.Brake);
    this.rightMotor.setNeutralMode(NeutralModeValue.Brake);

    this.elevatorEncoder = new Encoder(Constants.ElevatorConstants.elevatorEncoderChannelA,
        Constants.ElevatorConstants.elevatorEncoderChannelB);

    this.elevatorFeedforward = Constants.ElevatorConstants.lightLoadElevatorFeed;
    this.testPID = Constants.ElevatorConstants.lightLoadElevatorPID;

    this.elevatorEncoder.setReverseDirection(true);
    this.elevatorEncoder.reset();

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Elevator encoder", readEncoder());

    if (!RobotContainer.isTestMode().getAsBoolean() && true) {

      double calculatedSpeed = testPID.calculate((readEncoder() - targetHeightTicks) / 1000);
      double feedSpeed = elevatorFeedforward.calculate(leftMotor.get() / 10);

      SmartDashboard.putNumber("Elevator PID calc", calculatedSpeed);
      SmartDashboard.putNumber("Feed Elevator Calculation", feedSpeed);

      double finalSpeed = MathUtil.clamp(calculatedSpeed, -0.85, 0.95);

      leftMotor.set(finalSpeed);
      rightMotor.set(finalSpeed);
    }

    checkError();

  }

  // Manually set elevator speed with constant
  public InstantCommand getManualCommand(double speed) {
    return new InstantCommand(() -> {
      leftMotor.set(speed);
      rightMotor.set(speed);
    });
  }

  // Manually set elevator speed with supplier
  public Command getManualCommand(DoubleSupplier speedSupplier) {
    return Commands.run(() -> {
      leftMotor.set(speedSupplier.getAsDouble());
      rightMotor.set(speedSupplier.getAsDouble());
    });
  }

  // Retrieve raw encoder ticks
  public double readEncoder() {
    return elevatorEncoder.getRaw();
  }

  // Zero encoder
  private void resetEncoder() {
    elevatorEncoder.reset();
  }

  // Zero encoder command
  public Command getEncoderResetCommand() {
    return Commands.runOnce(this::resetEncoder);
  }

  // Setter for target height
  private void setTargetHeight(double value) {
    targetHeightTicks = value;
  }

  // Set target height in encoder ticks
  public Command setTargetHeightCommand(double value) {
    return Commands.runOnce(() -> {
      setTargetHeight(value);
    });
  }

  // Check for elevator error
  private void checkError() {
    // If elevator is below height 100, send error status
    if (readEncoder() < -100) {
      if (!RobotContainer.hasError(ErrorMode.ELEVATOR_REQUIRES_ZERO)) {
        RobotContainer.addError(new ErrorBody(ErrorMode.ELEVATOR_REQUIRES_ZERO, () -> {
          return readEncoder() > -100;
        }));
      }
    }
  }
}
