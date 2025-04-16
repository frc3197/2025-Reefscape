// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.enums.ErrorMode;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.util.ErrorBody;

@SuppressWarnings("unused")
public class Outtake extends SubsystemBase {

  private final LaserCan outtakeLaserCan;
  private final TalonFX outtakeMotor;
  private final BooleanSupplier detectsCoral;

  private final SparkMax starMotor;

  public Outtake() {

    outtakeLaserCan = new LaserCan(Constants.OuttakeConstants.outtakeLaserCan);

    outtakeMotor = new TalonFX(Constants.OuttakeConstants.outtakeMotorId);
    outtakeMotor.getConfigurator().apply(Constants.OuttakeConstants.outtakeMotorConfig);
    outtakeMotor.setNeutralMode(NeutralModeValue.Brake);

    detectsCoral = () -> {
      try {
        return  outtakeLaserCan.getMeasurement().distance_mm < Constants.OuttakeConstants.sensorRange;
      } catch (Exception e) {
        DriverStation.reportError("CORAL LASER CAN HAS LOST CAN OR POWER", false);
        if(!RobotContainer.hasError(ErrorMode.NO_LASER_CAN)) {
          RobotContainer.addError(new ErrorBody(ErrorMode.NO_LASER_CAN, this::laserCanFound));
        }
        return false;
      }
    };

    starMotor = new SparkMax(Constants.IntakeConstants.starMotorId, MotorType.kBrushless);

  }

  @Override
  public void periodic() {

    //SmartDashboard.putNumber("Sensor", outtakeLaserCan.getMeasurement().distance_mm);

    SmartDashboard.putBoolean("Has coral", detectsCoral.getAsBoolean());

    SmartDashboard.putNumber("match_time", DriverStation.getMatchTime());

  }

  public BooleanSupplier detectsCoralSupplier() {
    return detectsCoral;
  }

  public Command stopMotors() {
    return Commands.runOnce(() -> {
      feedOuttake(0);
    }, this);
  }

  public Command feedOuttake(double speed) {
    return Commands.runOnce(() -> {
      outtakeMotor.set(speed);
    }, this);
  }

  public Command setFeed(double value) {
    return Commands.runOnce(() -> {
      outtakeMotor.set(value);
    }, this);
  }

  public Command setStar(double value) {
    return Commands.runOnce(() -> {
      starMotor.set(value);
    });
  }

  private boolean laserCanFound() {
    try {
      return  outtakeLaserCan.getMeasurement().distance_mm < 100000000;
    } catch (Exception e) {
      return false;
    }
  }
}
