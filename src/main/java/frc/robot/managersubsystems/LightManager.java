// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managersubsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.ErrorMode;
import frc.robot.enums.AlertMode;

public class LightManager extends SubsystemBase {

  private AddressableLED led;
  private AddressableLEDBuffer buffer;

  public LightManager() {

    led = new AddressableLED(1);
    buffer = new AddressableLEDBuffer(60 + 102);
    led.setLength(buffer.getLength());

    // Set the data
    led.setData(buffer);
    led.start();
  }

  @Override
  public void periodic() {

    // Check for errors first
    if (RobotContainer.hasErrors()) {
      checkErrorLights();
      return;
    }

    if(RobotContainer.hasAlerts()) {
      checkAlertLights();
      return;
    }

    if (!RobotContainer.getEnabled()) {
      if (DriverStation.getAlliance().isEmpty()) {
        patternSetIdleOrange();
      } else {
        patternSetIdleAlliance();
      }
      return;
    }
    
    turnOffLights();
  }

  // Further logic to distinguish between priority levels

  private void checkErrorLights() {
    if (RobotContainer.hasError(ErrorMode.ELEVATOR_REQUIRES_ZERO)) {
      patternSetElevatorError();
    } else if (RobotContainer.hasError(ErrorMode.NO_LIMELIGHT)) {
      patternSetLimelightError();
    } else {
      patternSetOrangePiError();
    }
  }

  private void checkAlertLights() {
    if (RobotContainer.hasAlert(AlertMode.ACQUIRED_CORAL)) {
      patternSetAcquiredCoralAlert();
    }

  }

  // Actual light patterns
  private void patternSetIdleAlliance() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      if (RobotContainer.isRed())
        buffer.setRGB(i, 0, 130 + (int) Math.round(Math.cos((Timer.getTimestamp() * 1.25 + (i / 6)) * 2.75) * 125.0),
            0);
      else
        buffer.setRGB(i, 0, 0,
            130 + (int) Math.round(Math.cos((Timer.getTimestamp() * 1.25 + (i / 6)) * 2.75) * 125.0));
    }
    led.setData(buffer);
  }

  private void patternSetIdleOrange() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 120 + (int) Math.round(Math.cos((Timer.getTimestamp() * 1.25 + (i / 6)) * 2.75) * 125.0), 120 + (int) Math.round(Math.cos((Timer.getTimestamp() * 1.25 + (i / 6)) * 2.75) * 125.0),
          0);
    }
    led.setData(buffer);
  }

  private void patternSetElevatorError() {
    if (Timer.getTimestamp() % 2 <= 1)
      for (int i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the GRB values for red
        buffer.setRGB(i, 0, 180, 0);
      }
    else {
      for (int i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the GRB values for red
        buffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(buffer);
  }

  private void patternSetAcquiredCoralAlert() {
    if (Timer.getTimestamp() % 0.12 <= 0.06)
      for (int i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the GRB values for red
        buffer.setRGB(i, 255, 0, 0);
      }
    else {
      for (int i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the GRB values for red
        buffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(buffer);
  }

  private void patternSetLimelightError() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 90, 0, 0);
    }
    led.setData(buffer);
  }

  private void patternSetOrangePiError() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 200, 180, 0);
    }
    led.setData(buffer);
  }

  private void turnOffLights() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 0, 0, 0);
    }
    led.setData(buffer);
  }
}
