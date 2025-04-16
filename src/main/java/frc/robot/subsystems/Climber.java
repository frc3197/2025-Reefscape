// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.RobotMode;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private TalonFX climberArm;

  private DigitalInput limit = new DigitalInput(4);

  public Climber() {
    this.climberArm = new TalonFX(16);

    CurrentLimitsConfigs climberArmConfig = new CurrentLimitsConfigs().withStatorCurrentLimit(55).withStatorCurrentLimitEnable(true).withSupplyCurrentLimit(25).withSupplyCurrentLimitEnable(true);

    this.climberArm.getConfigurator().apply(climberArmConfig);

    this.climberArm.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LIMIT CLIMB", !limit.get());
    if(!limit.get() && RobotContainer.getRobotMode() != RobotMode.FULLY_CLIMBED) {
      RobotContainer.setRobotMode(RobotMode.FULLY_CLIMBED);
    } else {
      if(limit.get() && RobotContainer.getRobotMode() == RobotMode.FULLY_CLIMBED && !RobotContainer.getEnabled()) {
        RobotContainer.setRobotMode(RobotMode.NONE);
      }
    }
  }

  public Command setClimberSpeedValue(double speed) {
    return Commands.runOnce(() -> {
      climberArm.set(speed);
    });
  }
  
  public Command setClimberArmSpeed(Supplier<Double> speed) {
    return Commands.run(() -> {
      if(speed.get() < 0) {
        climberArm.set(speed.get());
      } else {
        if (!limit.get()) {
          climberArm.set(0);
        } else {
          climberArm.set(speed.get());
        }
      }
    });
  }
}
