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
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.AlertMode;
import frc.robot.enums.ErrorMode;
import frc.robot.util.AlertBody;
import frc.robot.util.ErrorBody;


@SuppressWarnings("unused")
public class Algae extends SubsystemBase {

  // Deploy motor
  private final TalonFX deployMotor;

  // Grabber motors
  private final SparkMax leftGrabberMotor;
  private final SparkMax rightGrabberMotor;

  // Grabber configs
  private final SparkMaxConfig leftConfig;
  private final SparkMaxConfig rightConfig;

  // Algae detection sensor
  private final LaserCan algaeLaserCan;

  // Sensor detects algae
  private boolean hasAlgae = false;

  // Algae encoder
  private final DutyCycleEncoder algaeEncoder;

  // PID and feed forward
  private final PIDController emptyArmPID;
  private final ArmFeedforward emptyArmFeedForward;
  private final PIDController algaeArmPID;
  private final ArmFeedforward algaeArmFeedForward;
  private double targetAlgaeAngle = 85;

  public Algae() {

    this.deployMotor = new TalonFX(Constants.AlgaeConstants.deployMotorId);
    this.deployMotor.getConfigurator().apply(Constants.AlgaeConstants.deployMotorConfig);
    deployMotor.setNeutralMode(NeutralModeValue.Brake);

    this.leftGrabberMotor = new SparkMax(Constants.AlgaeConstants.leftGrabberMotorId, MotorType.kBrushless);
    this.rightGrabberMotor = new SparkMax(Constants.AlgaeConstants.rightGrabberMotorId, MotorType.kBrushless);

    // Configuration for spark maxes
    this.leftConfig = new SparkMaxConfig();
    this.leftConfig.smartCurrentLimit(40, 40);
    this.leftConfig.inverted(false);
    this.leftConfig.idleMode(IdleMode.kBrake);
    this.leftGrabberMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configuration for spark maxes
    this.rightConfig = new SparkMaxConfig();
    this.rightConfig.smartCurrentLimit(40, 40);
    this.rightConfig.inverted(true);
    this.rightConfig.idleMode(IdleMode.kBrake);
    this.rightGrabberMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.algaeLaserCan = new LaserCan(Constants.AlgaeConstants.algaeLaserCanId);
    this.algaeEncoder = new DutyCycleEncoder(Constants.AlgaeConstants.algaeEncoderChannel);

    this.emptyArmFeedForward = Constants.AlgaeConstants.emptyLoadArmFeedForward;
    this.algaeArmFeedForward = Constants.AlgaeConstants.algaeLoadArmFeedForward;

    this.emptyArmPID = Constants.AlgaeConstants.emptyLoadArmPID;
    this.algaeArmPID = Constants.AlgaeConstants.algaeLoadArmPID;

  }

  public Command setTargetAngleDegrees(double angleDegrees) {
    return Commands.runOnce(() -> {
      this.targetAlgaeAngle = angleDegrees;
    }, this);
  }

  // Manually set grabber speed
  public Command setAlgaeGrabberSpeedCommand(double speed) {
    return Commands.runOnce(() -> {
      leftGrabberMotor.set(speed);
      rightGrabberMotor.set(speed);
    }, this);
  }

  public Command setAlgaeGrabberSpeedCommand(double speedLeft, double speedRight) {
    return Commands.runOnce(() -> {
      leftGrabberMotor.set(speedLeft);
      rightGrabberMotor.set(speedRight);
    }, this);
  }

  // Manually set deploy speed
  public Command setDeploySpeedCommand(double speed) {
    return Commands.runOnce(() -> {
      deployMotor.set(speed);
    }, this);
  }

  public BooleanSupplier getHasAlgaeSupplier() {
    return hasAlgae();
  }

  public Command getAutomaticAlgaeCommand() {
    return Commands.runOnce(() -> {
      leftGrabberMotor.set(0.85);
      rightGrabberMotor.set(0.85);
    }, this).andThen(Commands.waitUntil(hasAlgae()).withTimeout(5)).andThen(() -> {
      RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_ALGAE, 1.25));
      leftGrabberMotor.set(0);
      rightGrabberMotor.set(0);
    }, this);
  }

  public BooleanSupplier hasAlgae() {
    return (() -> {
      try {
        return algaeLaserCan.getMeasurement().distance_mm < Constants.AlgaeConstants.algaeSensorDistance
            && algaeLaserCan.getMeasurement().distance_mm >= 1;
      } catch (Exception e) {
        DriverStation.reportError("ALGAE LASER CAN HAS LOST CAN OR POWER", false);
        if(!RobotContainer.hasError(ErrorMode.NO_LASER_CAN)) {
          RobotContainer.addError(new ErrorBody(ErrorMode.NO_LASER_CAN, this::laserCanFound));
        }
        return false;
      }
    });
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Algae Arm Encoder", algaeEncoder.get());
    SmartDashboard.putNumber("Algae Arm Angle", convertTicksToDegrees(algaeEncoder.get()));
    // SmartDashboard.putNumber("Algae Laser Can",
    // algaeLaserCan.getMeasurement().distance_mm);

    updateAlgaeStatus();

    if (!RobotContainer.getTestMode()) {
      updateClosedLoop();
    } else {
      deployMotor.set(0);
    }

    // 0.41

    // deployMotor.setVoltage(0.42);

  }

  // Check if algae status has changed
  private void updateAlgaeStatus() {

    if (hasAlgae().getAsBoolean() != hasAlgae) {
      hasAlgae = hasAlgae().getAsBoolean();
      RobotContainer.setHasAlgae(hasAlgae().getAsBoolean());

      if (hasAlgae().getAsBoolean()) {
        // RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_ALGAE, 1.25));
      }
    }

  }

  // Returns arm angle in degrees from encoder ticks
  public double convertTicksToDegrees(double ticks) {
    return 90 - ((ticks - Constants.AlgaeConstants.algaeUpEncoder) * 90.0
        / (Constants.AlgaeConstants.algaeDownEncoder - Constants.AlgaeConstants.algaeUpEncoder));
    // return ((Constants.AlgaeConstants.algaeDownEncoder - ticks) /
    // Constants.AlgaeConstants.algaeUpEncoder) * (90);
  }

  private void updateClosedLoop() {

    SmartDashboard.putNumber("Angle Algae OFFSET", targetAlgaeAngle + convertTicksToDegrees(algaeEncoder.get()));

    double feedForwardSpeed = 0;
    double pidSpeed = 0;

    double armAngleDegrees = convertTicksToDegrees(algaeEncoder.get());
    double armAngleRadians = Units.degreesToRadians(armAngleDegrees);

    double rawVelocity = deployMotor.getVelocity().getValueAsDouble();
    double radianVelocity = Units.rotationsToRadians(rawVelocity);

    if (hasAlgae().getAsBoolean() && false) {
      feedForwardSpeed = algaeArmFeedForward.calculate(armAngleRadians, radianVelocity);
      pidSpeed = algaeArmPID.calculate(targetAlgaeAngle - algaeEncoder.get());
    } else {
      feedForwardSpeed = emptyArmFeedForward.calculate(armAngleRadians, -radianVelocity);
      pidSpeed = -emptyArmPID.calculate((targetAlgaeAngle - convertTicksToDegrees(algaeEncoder.get())) / 10);
    }

    double combinedSpeed = MathUtil.clamp(pidSpeed, -0.4, 0.4);
    deployMotor.set(combinedSpeed + feedForwardSpeed);
  }

  private boolean laserCanFound() {
    // SmartDashboard.putNumber("Timestamp", getLimelightTime(bestPose));
    // SmartDashboard.putNumber("Limelight dist", bestPose.avgTagDist);

    try {
      return  algaeLaserCan.getMeasurement().distance_mm < 100000000;
    } catch (Exception e) {
      return false;
    }
  }
}
