// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managersubsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.ErrorMode;
import frc.robot.enums.RobotMode;
import frc.robot.enums.AlertMode;

@SuppressWarnings("unused")
public class LightManager extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LightManager() {

    this.led = new AddressableLED(1);
    this.buffer = new AddressableLEDBuffer(60 + 102 + 15 + 28 + 9 + 1);
    this.led.setLength(buffer.getLength());

    // Set the data
    this.led.setData(buffer);
    this.led.start();
  }

  @Override
  public void periodic() {

    if (RobotContainer.getEnabled() && DriverStation.getMatchTime() <= 5.0 && !DriverStation.isAutonomous()) {
      // setCountdownLights();
      // return;
    }

    // Check for errors first
    if (RobotContainer.hasErrors()) {
      checkErrorLights();
      return;
    }

    SmartDashboard.putBoolean("Has Alert", RobotContainer.hasAlert(AlertMode.FULLY_ALIGNED));
    if (RobotContainer.hasAlerts()) {
      checkAlertLights();
      return;
    }

    if (RobotContainer.getEnabled() && RobotContainer.getRobotMode() != RobotMode.NONE) {
      checkModeLights();
      return;
    } else {
      SmartDashboard.putString("Robot Mode", "NONE");
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
    } else if(RobotContainer.hasError(ErrorMode.NO_LASER_CAN)) {
      patternSetLaserCanCoralError();
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
    if (Timer.getTimestamp() % 2 <= 1)
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 90, 0, 0);
      }
    else {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(buffer);
  }

  private void patternSetLaserCanCoralError() {
    if (Timer.getTimestamp() % 2 <= 1)
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 155, 155, 0);
      }
    else {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 0, 0, 0);
      }
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
        buffer.setRGB(i, 150, 255, 0);
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

  // Orange climb pattern
  private void patternSetFullyClimbed() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red

      double intensity = (Math.sin((i / 3.0) + (Timer.getTimestamp() * 7.0)));

      if (intensity < 0) {
        intensity = 0;
      }

      buffer.setRGB(i, (int) (80.0 * intensity), (int) (255.0 * intensity), 0);
      // buffer.setRGB(i, (int) (80.0 * intensity), (int) (255.0 * intensity), 0);
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

  private void patternSetSeesAlgae() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 20, 0, 0);
    }
    led.setData(buffer);
  }

  private void patternSetSeesNoAlgae() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 0, 200, 0);
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

  private void patternSetAlignBlue() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      buffer.setRGB(i, 60, 75, 200);
    }
    led.setData(buffer);
  }

  private void patternSetAlignNet() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red

      double intensity = (Math.sin((i / 5.0) + (Timer.getTimestamp() * 3.0)));

      intensity = MathUtil.clamp(intensity, 0, 1);

      buffer.setRGB(i, (int) (80.0 * intensity), 0, (int) (255.0 * (1 - intensity)));
    }
    led.setData(buffer);
  }

  private void checkModeLights() {
    if (RobotContainer.getRobotMode() == RobotMode.NET_ALIGNED) {
      patternSetAlignNet();
      SmartDashboard.putString("Robot Mode", "NET ALIGN");
      return;
    } else if (RobotContainer.getRobotMode() == RobotMode.ALIGN_REEF_CUSTOM) {
      patternSetAlignCustom();
      SmartDashboard.putString("Robot Mode", "REEF ALIGN");
      return;
    } else if (RobotContainer.getRobotMode() == RobotMode.DETECTS_PIECE && DriverStation.isAutonomous()) {
      patternSetAlignBlue();
      SmartDashboard.putString("Robot Mode", "DETECTS PIECE");
      return;
    } else if (RobotContainer.getRobotMode() == RobotMode.FULLY_CLIMBED) {
      patternSetFullyClimbed();
      SmartDashboard.putString("Robot Mode", "FULLY CLIMBED");
      return;
    } else if(RobotContainer.getRobotMode() == RobotMode.SEES_ALGAE) {
      patternSetSeesAlgae();
    } else if(RobotContainer.getRobotMode() == RobotMode.SEES_NO_ALGAE) {
      patternSetSeesNoAlgae();
    } else {
      turnOffLights();
      SmartDashboard.putString("Robot Mode", "NONE");
      return;
    }
  }

  private void setCountdownLights() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red
      if (i > buffer.getLength() - (int) Math.floor((DriverStation.getMatchTime() / 5.0) * (double) buffer.getLength())
          - 1) {
        buffer.setRGB(i, 80, 255, 35);
      } else {
        buffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(buffer);
  }
}
