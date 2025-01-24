// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {

  private final TimeOfFlight outtakeSensor;

  private final TalonFX outtakeMotor;

  private final BooleanSupplier coralBridging;
  private final BooleanSupplier notCoralBridging;

  public Outtake() {

    outtakeSensor = new TimeOfFlight(0);
    outtakeSensor.setRangingMode(RangingMode.Short, 1);

    outtakeMotor = new TalonFX(Constants.OuttakeConstants.outtakeMotorId);
    outtakeMotor.getConfigurator().apply(Constants.OuttakeConstants.outtakeMotorConfig);

    coralBridging = () -> {
      return outtakeSensor.getRange() < Constants.OuttakeConstants.sensorRange;
    };

    notCoralBridging = () -> {
      return outtakeSensor.getRange() > Constants.OuttakeConstants.sensorRange;
    };

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("ENCODER", outtakeSensor.getRange());

    SmartDashboard.putBoolean("Not bridging", !coralBridging.getAsBoolean());

  }

  public Command feedOuttake1() {
    return Commands.run(() -> {
      outtakeMotor.set(Constants.OuttakeConstants.outtakeFeedSpeed);
    }, this).until(coralBridging).andThen(new WaitCommand(5).until(() -> {
      return !coralBridging.getAsBoolean();
    }));
  }

  public Command feedOuttake2() {
    return Commands.run(() -> {
      outtakeMotor.set(Constants.OuttakeConstants.outtakeFeedSpeed);
    }, this).until(notCoralBridging)
        .andThen(() -> outtakeMotor.set(0)).withName("Feed Outtake Command");
  }

  public Command spitOuttake() {
    return Commands.runOnce(() -> {
      outtakeMotor.set(Constants.OuttakeConstants.outtakeSpitSpeed);
    }, this).andThen(new WaitCommand(1)).andThen(() -> outtakeMotor.set(0)).withName("Spit Outtake");
  }

  public Command setFeed(double value) {
    return Commands.runOnce(() -> {
      outtakeMotor.set(value);
    });
  }
}
