// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algae extends SubsystemBase {

  private final TalonFX deployMotor;
  private final TalonFX spinMotor;

  public Algae() {
    deployMotor = new TalonFX(Constants.AlgaeConstants.deployId);
    spinMotor = new TalonFX(Constants.AlgaeConstants.spinId);

    deployMotor.getConfigurator().apply(Constants.AlgaeConstants.deployConfig);
    spinMotor.getConfigurator().apply(Constants.AlgaeConstants.spinConfig);
  }

  
}
