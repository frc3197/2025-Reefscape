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
import frc.robot.enums.RobotMode;
import frc.robot.enums.AlertMode;

public class LightManager extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LightManager() {

    this.led = new AddressableLED(1);
    this.buffer = new AddressableLEDBuffer(60 + 102);
    this.led.setLength(buffer.getLength());

    // Set the data
    this.led.setData(buffer);
    this.led.start();
  }

  @Override
  public void periodic() {

    // Check for errors first
    if (RobotContainer.hasErrors()) {
      checkErrorLights();
      return;
    }

    if (RobotContainer.hasAlerts()) {
      checkAlertLights();
      return;
    }

    if (RobotContainer.getEnabled() && RobotContainer.getRobotMode() != RobotMode.NONE) {
      checkModeLights();
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

  // --------------------------------------------------------------------
  // Further logic to distinguish between priority levels
  // --------------------------------------------------------------------

  // Check error status
  private void checkErrorLights() {
    if (RobotContainer.hasError(ErrorMode.ELEVATOR_REQUIRES_ZERO)) {
      patternSetElevatorError();
    } else if (RobotContainer.hasError(ErrorMode.NO_LIMELIGHT)) {
      patternSetLimelightError();
    } else {
      patternSetOrangePiError();
    }
  }

  // Check alert status
  private void checkAlertLights() {

    if (!RobotContainer.getEnabled()) {
      return;
    }
    if (RobotContainer.hasAlert(AlertMode.FULLY_ALIGNED)) {
      patternSetFullyAligned();
      return;
    }
    if (RobotContainer.hasAlert(AlertMode.ACQUIRED_CORAL)) {
      patternSetAcquiredCoralAlert();
      return;
    }
    if (RobotContainer.hasAlert(AlertMode.ACQUIRED_ALGAE)) {
      patternSetAcquiredAlgaeAlert();
      return;
    }
  }

  // --------------------------------------------------------------------
  // ERROR PATTERNS
  // --------------------------------------------------------------------

  // Elevator error
  private void patternSetElevatorError() {
    if (Timer.getTimestamp() % 2 <= 1)
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 0, 180, 0);
      }
    else {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(buffer);
  }

  // Limelight error
  private void patternSetLimelightError() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 90, 0, 0);
    }
    led.setData(buffer);
  }

  // Orange pi error
  private void patternSetOrangePiError() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 200, 180, 0);
    }
    led.setData(buffer);
  }

  // --------------------------------------------------------------------
  // ALERT PATTERNS
  // --------------------------------------------------------------------

  // Coral alert
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

  private void patternSetFullyAligned() {
    if (Timer.getTimestamp() % 0.12 <= 0.06)
      for (int i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the GRB values for red
        buffer.setRGB(i, 150, 150, 150);
      }
    else {
      for (int i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the GRB values for red
        buffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(buffer);
  }

  // Algae alert
  private void patternSetAcquiredAlgaeAlert() {
    if (Timer.getTimestamp() % 0.12 <= 0.06)
      for (int i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the GRB values for red
        buffer.setRGB(i, 255, 0, 200);
      }
    else {
      for (int i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the GRB values for red
        buffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(buffer);
  }

  // --------------------------------------------------------------------
  // IDLE PATTERNS
  // --------------------------------------------------------------------

  // Alliance pattern
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

  // Orange pattern
  private void patternSetIdleOrange() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red

      double intensity = (Math.sin((i / 5.0) + (Timer.getTimestamp() / 1.0)));

      if (intensity < 0) {
        intensity = 0;
      }

      buffer.setRGB(i, (int) (80.0 * intensity), (int) (255.0 * intensity), 0);
    }
    led.setData(buffer);
  }

  // Turn off lights
  private void turnOffLights() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 0, 0, 0);
    }
    led.setData(buffer);
  }

  private void patternSetAlignRough() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 10, 100, 10);
    }
    led.setData(buffer);
  }

  private void patternSetAlignFine() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 120, 250, 120);
    }
    led.setData(buffer);
  }

  private void patternSetAlignCustom() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 10, 120, 35);
    }
    led.setData(buffer);
  }

  private void checkModeLights() {
    if (RobotContainer.getRobotMode() == RobotMode.ALIGN_REEF_ROUGH) {
      patternSetAlignRough();
      return;
    } else if (RobotContainer.getRobotMode() == RobotMode.ALIGN_REEF_FINE) {
      patternSetAlignFine();
      return;
    } else if (RobotContainer.getRobotMode() == RobotMode.ALIGN_REEF_CUSTOM) {
      patternSetAlignCustom();
      return;
    } else {
      return;
    }
  }
}
