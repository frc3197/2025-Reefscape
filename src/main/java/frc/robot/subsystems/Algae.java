// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.AlertMode;
import frc.robot.util.AlertBody;

public class Algae extends SubsystemBase {

  private final TalonFX deployMotor;

  private final SparkMax leftGrabberMotor;
  private final SparkMax rightGrabberMotor;

  private final LaserCan algaeLaserCan;

  private SparkMaxConfig leftConfig;
  private SparkMaxConfig rightConfig;

  private boolean hasAlgae = false;

  private final DutyCycleEncoder algaeEncoder;

  private final PIDController algaeArmPID = new PIDController(1, 0, 0);
  private double targetAlgaeAngle = 0.3;

  public Algae() {

    this.deployMotor = new TalonFX(Constants.AlgaeConstants.deployMotorId);
    this.deployMotor.getConfigurator().apply(Constants.AlgaeConstants.deployMotorConfig);
    deployMotor.setNeutralMode(NeutralModeValue.Brake);

    this.leftGrabberMotor = new SparkMax(Constants.AlgaeConstants.leftGrabberMotorId, MotorType.kBrushless);
    this.rightGrabberMotor = new SparkMax(Constants.AlgaeConstants.rightGrabberMotorId, MotorType.kBrushless);

    this.leftConfig = new SparkMaxConfig();
    this.leftConfig.smartCurrentLimit(20, 20);
    this.leftConfig.inverted(false);
    this.leftConfig.idleMode(IdleMode.kBrake);
    this.leftGrabberMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.rightConfig = new SparkMaxConfig();
    this.rightConfig.smartCurrentLimit(20, 20);
    this.rightConfig.inverted(true);
    this.rightConfig.idleMode(IdleMode.kBrake);
    this.rightGrabberMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.algaeLaserCan = new LaserCan(Constants.AlgaeConstants.algaeLaserCanId);

    this.algaeEncoder = new DutyCycleEncoder(Constants.AlgaeConstants.algaeEncoderChannel);
  }

  public Command setTargetAngle(double angle) {
    return Commands.runOnce(() -> {
      this.targetAlgaeAngle = angle;
    });
  }

  public Command setAlgaeGrabbers(double speed) {
    return Commands.runOnce(() -> {
      leftGrabberMotor.set(speed);
      rightGrabberMotor.set(speed);
    }, this);
  }

  public Command setDeploySpeed(double speed) {
    return Commands.runOnce(() -> {
      deployMotor.set(speed);
    }, this);
  }

  public BooleanSupplier hasAlgae() {
    return (() -> {
      return algaeLaserCan.getMeasurement().distance_mm < Constants.AlgaeConstants.algaeSensorDistance
          && algaeLaserCan.getMeasurement().distance_mm >= 1;
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Encoder", algaeEncoder.get());
    SmartDashboard.putNumber("Algae Laser Can", algaeLaserCan.getMeasurement().distance_mm);

    updateAlgaeStatus();

    if (true) {
      double speed = algaeArmPID.calculate(targetAlgaeAngle - algaeEncoder.get()) * 2.5;
      speed = MathUtil.clamp(speed, -0.4, 0.4);
      deployMotor.set(speed);
    }
  }

  private void updateAlgaeStatus() {
    if (hasAlgae().getAsBoolean() != hasAlgae) {
      hasAlgae = hasAlgae().getAsBoolean();

      if (hasAlgae().getAsBoolean()) {
        RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_ALGAE, 1.25));
      }
    }
  }

}
