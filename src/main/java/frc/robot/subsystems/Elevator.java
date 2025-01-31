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

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final Encoder elevatorEncoder;

  private ElevatorFeedforward elevatorFeedforward;
  private final PIDController testPID;

  private double targetHeight = 0;

  public Elevator() {
    leftMotor = new TalonFX(Constants.ElevatorConstants.leftElevatorMotorId);
    rightMotor = new TalonFX(Constants.ElevatorConstants.rightElevatorMotorId);

    leftMotor.getConfigurator().apply(Constants.ElevatorConstants.leftElevatorMotorConfig);
    rightMotor.getConfigurator().apply(Constants.ElevatorConstants.rightElevatorMotorConfig);

    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    elevatorEncoder = new Encoder(Constants.ElevatorConstants.elevatorEncoderChannelA,
        Constants.ElevatorConstants.elevatorEncoderChannelB);

    elevatorFeedforward = Constants.ElevatorConstants.lightLoadElevatorFeed;
    testPID = Constants.ElevatorConstants.lightLoadElevatorPID;

    elevatorEncoder.setReverseDirection(true);
    elevatorEncoder.reset();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Elevator encoder", readEncoder());
    SmartDashboard.putNumber("Elevator height", getElevatorHeight());

    if (!RobotContainer.isTestMode().getAsBoolean() && true) {

      double calculatedSpeed = testPID.calculate((getElevatorHeight() - targetHeight)/1000);
      double feedSpeed = elevatorFeedforward.calculate(leftMotor.get() / 10);

      SmartDashboard.putNumber("Elevator PID calc", calculatedSpeed);
      SmartDashboard.putNumber("Feed Elevator Calculation", feedSpeed);

      double finalSpeed = MathUtil.clamp(calculatedSpeed, -0.95, 0.95);

      leftMotor.set(finalSpeed);
      rightMotor.set(finalSpeed);
    }

    // Check for elevator errors
    if (readEncoder() < -100) {
      if (!RobotContainer.hasError(ErrorMode.ELEVATOR_REQUIRES_ZERO)) {
        RobotContainer.addError(new ErrorBody(ErrorMode.ELEVATOR_REQUIRES_ZERO, () -> {
          return readEncoder() > -100;
        }));
      }
    }

  }

  public InstantCommand getManualCommand(double speed) {
    return new InstantCommand(() -> {
      leftMotor.set(speed);
      rightMotor.set(speed);
    });
  }

  public Command getManualSupplierCommand(DoubleSupplier speedSupplier) {
    return Commands.run(() -> {
      leftMotor.set(speedSupplier.getAsDouble());
      rightMotor.set(speedSupplier.getAsDouble());
    });
  }

  public double readEncoder() {
    return elevatorEncoder.getRaw();
  }

  private void resetEncoder() {
    elevatorEncoder.reset();
  }

  public Command getEncoderResetCommand() {
    return Commands.runOnce(this::resetEncoder);
  }

  public double getElevatorHeight() {
    return ((readEncoder() / Constants.ElevatorConstants.encoderHighValue)
        * (Constants.ElevatorConstants.elevatorHighHeight - Constants.ElevatorConstants.elevatorLowHeight))
        + Constants.ElevatorConstants.elevatorLowHeight;
  }

  public void setTargetHeight(double value) {
    targetHeight = value;
  }

  public Command setTargetHeightCommand(double value) {
    return Commands.runOnce(() -> {
      setTargetHeight(value);
    });
  }
}
