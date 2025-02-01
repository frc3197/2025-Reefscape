// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class Intake extends SubsystemBase {

  //private final TalonFX leftIntakeMotor;
  //private final TalonFX rightIntakeMotor;

  /** Creates a new Intake. */
  public Intake() {

    //leftIntakeMotor = new TalonFX(Constants.IntakeConstants.leftIntakeMotorId);
    //rightIntakeMotor = new TalonFX(Constants.IntakeConstants.rightIntakeMotorId);

    //leftIntakeMotor.getConfigurator().apply(Constants.IntakeConstants.leftIntakeMotorConfig);
    //rightIntakeMotor.getConfigurator().apply(Constants.IntakeConstants.rightIntakeMotorConfig);

  }

  @Override
  public void periodic() {
  }

  public Command setIntakeCommand(double leftSpeed, double rightSpeed) {
    return Commands.runOnce(() -> {
      //leftIntakeMotor.set(leftSpeed);
      //rightIntakeMotor.set(rightSpeed);
    }, this).withName("Set Intake");
  }

  public Command getIntakeStopCommand() {
    return Commands.runOnce(() -> {
      //leftIntakeMotor.set(0);
      //rightIntakeMotor.set(0);
    }, this).withName("Stop Intake");
  }
}
