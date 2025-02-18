// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private TalonFX climberArm;

  public Climber() {
    this.climberArm = new TalonFX(16);

    CurrentLimitsConfigs climberArmConfig = new CurrentLimitsConfigs().withStatorCurrentLimit(100).withStatorCurrentLimitEnable(true).withSupplyCurrentLimit(100).withSupplyCurrentLimitEnable(true);

    this.climberArm.getConfigurator().apply(climberArmConfig);

    this.climberArm.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setClimberArmSpeed(double speed) {
    return Commands.runOnce(() -> {
      climberArm.set(speed*1);
    });
  }
  
  public Command setClimberArmSpeed(Supplier<Double> speed) {
    return Commands.runOnce(() -> {
      climberArm.set(speed.get()*1);
    });
  }
}
