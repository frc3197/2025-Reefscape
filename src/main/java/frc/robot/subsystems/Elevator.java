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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {

  // Elevation motors
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  // Elevation height encoder
  private final Encoder elevatorEncoder;

  // Elevator
  private ElevatorFeedforward emptyElevatorFeedforward;
  private final ProfiledPIDController emptyLoadPID;

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

    this.emptyElevatorFeedforward = Constants.ElevatorConstants.emptyLoadElevatorFeed;

    this.emptyLoadPID = Constants.ElevatorConstants.emptyLoadElevatorPID;

    this.elevatorEncoder.setReverseDirection(true);
    this.elevatorEncoder.reset();

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Test Mode", RobotContainer.getTestMode());
    SmartDashboard.putNumber("NEW ELEVATOR ENCODER", readEncoder());

    // If not in test mode, update closed loop
    if (!RobotContainer.getTestMode()) {
      updateClosedLoop();
    }

    // Check elevator errors
    checkError();
  }

  public double getTargetHeight() {
    return targetHeightTicks;
  }

  public double getPositionError() {
    return Math.abs(readEncoder() - targetHeightTicks);
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
    return elevatorEncoder.getRaw()/1000.0;
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
    emptyLoadPID.setGoal(value);
  }

  // Set target height in encoder ticks
  public Command setTargetHeightCommand(double value) {
    return Commands.runOnce(() -> {
      setTargetHeight(value);
      emptyLoadPID.setGoal(value);
    });
  }

  // Set target height in encoder ticks
  public Command setTargetHeightCommand(DoubleSupplier value) {
    return Commands.runOnce(() -> {
      setTargetHeight(value.getAsDouble());
      emptyLoadPID.setGoal(value.getAsDouble());
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

  public void setGoal(double height, double velocity) {
    emptyLoadPID.setGoal(new TrapezoidProfile.State(height, velocity));
}

  // Update the closed loop for automatic control
  private void updateClosedLoop() {

    double calculatedPIDSpeed = 0;
    double feedForwardSpeed = 0;

    feedForwardSpeed = emptyElevatorFeedforward.calculate(emptyLoadPID.getSetpoint().velocity);
    calculatedPIDSpeed = emptyLoadPID.calculate(readEncoder());

    //calculatedPIDSpeed *= 0.015;

    SmartDashboard.putNumber("Elevator PID calc", calculatedPIDSpeed);
    SmartDashboard.putNumber("Elevator Speed", leftMotor.getVelocity(true).getValueAsDouble());
    SmartDashboard.putNumber("Elevator Velocity NEW", emptyLoadPID.getSetpoint().velocity);
    SmartDashboard.putNumber("Feed Elevator Calculation", feedForwardSpeed);
    
    double finalSpeed = MathUtil.clamp((calculatedPIDSpeed * 1.05) + (feedForwardSpeed*1), -0.25, 1.0);
    SmartDashboard.putNumber("Elevator FINAL", finalSpeed);

    finalSpeed = MathUtil.applyDeadband(finalSpeed, 0.09);
    
    leftMotor.set(finalSpeed);
    rightMotor.set(finalSpeed);
  }
}
