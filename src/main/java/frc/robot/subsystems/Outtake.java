// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class Outtake extends SubsystemBase {

  private LaserCan outtakeLaserCan;
  private final TalonFX outtakeMotor;
  private final BooleanSupplier detectsCoral;

  public Outtake() {

    outtakeLaserCan = new LaserCan(Constants.OuttakeConstants.outtakeLaserCan);

    outtakeMotor = new TalonFX(Constants.OuttakeConstants.outtakeMotorId);
    outtakeMotor.getConfigurator().apply(Constants.OuttakeConstants.outtakeMotorConfig);
    outtakeMotor.setNeutralMode(NeutralModeValue.Brake);

    detectsCoral = () -> {
      return outtakeLaserCan.getMeasurement().distance_mm < Constants.OuttakeConstants.sensorRange;
    };

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Sensor", outtakeLaserCan.getMeasurement().distance_mm);

    SmartDashboard.putBoolean("Has coral", detectsCoral.getAsBoolean());

    SmartDashboard.putNumber("match_time", DriverStation.getMatchTime());

  }

  public BooleanSupplier detectsCoralSupplier() {
    return detectsCoral;
  }

  public Command stopMotors() {
    return Commands.runOnce(() -> {
      feedOuttake(0);
    });
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
