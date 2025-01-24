// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managersubsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.LightPattern;

public class LightManager extends SubsystemBase {

  private AddressableLED led;
  private AddressableLEDBuffer buffer;
  private Timer timer = new Timer();

  private LightPattern pattern = LightPattern.IdleAlliance;

  public LightManager() {

    led = new AddressableLED(1);
    buffer = new AddressableLEDBuffer(60 + 102);
    led.setLength(buffer.getLength());

    // Set the data
    led.setData(buffer);
    led.start();

    timer.start();
  }

  @Override
  public void periodic() {
    switch (pattern) {
      case IdleAlliance:
        setIdleAlliance();
        break;

      default:
        break;
    }
  }

  public void setIdleAlliance() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      if (RobotContainer.isRed())
        buffer.setRGB(i, 0, 130 + (int) Math.round(Math.cos((timer.get() * 1.25 + (i / 6)) * 2.75) * 125.0), 0);
      else
        buffer.setRGB(i, 0, 0, 130 + (int) Math.round(Math.cos((timer.get() * 1.25 + (i / 6)) * 2.75) * 125.0));
    }
    led.setData(buffer);
  }

  public void setLightPattern(LightPattern pattern) {
    this.pattern = pattern;
  }
}
