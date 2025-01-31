// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {

  private LaserCan outtakeLaserCan;
  private final TalonFX outtakeMotor;
  private final BooleanSupplier coralBridging;

  public Outtake() {

    outtakeLaserCan = new LaserCan(Constants.OuttakeConstants.outtakeLaserCan);

    outtakeMotor = new TalonFX(Constants.OuttakeConstants.outtakeMotorId);
    outtakeMotor.getConfigurator().apply(Constants.OuttakeConstants.outtakeMotorConfig);

    coralBridging = () -> {
      return outtakeLaserCan.getMeasurement().distance_mm < Constants.OuttakeConstants.sensorRange;
    };

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Sensor", outtakeLaserCan.getMeasurement().distance_mm);

    SmartDashboard.putBoolean("Not bridging", !coralBridging.getAsBoolean());

  }

  public boolean isBridging() {
    return coralBridging.getAsBoolean();
  }

  public BooleanSupplier isBridgingSupplier() {
    return coralBridging;
  }

  public Command feedOuttake(double speed) {
    return Commands.runOnce(() -> {
      outtakeMotor.set(speed);
    }, this);
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
