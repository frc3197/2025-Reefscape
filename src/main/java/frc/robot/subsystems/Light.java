// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Light extends SubsystemBase {

  private AddressableLED led;
  private AddressableLEDBuffer buffer;
  private Timer timer = new Timer();

  public Light() {
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
    setIdleAlliance();
  }

  public void setIdleAlliance() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      if (RobotContainer.isRed())
        buffer.setRGB(i, 0, 130 + (int)Math.round(Math.cos((timer.get()*1.25 + (i / 6)) * 2.75) * 125.0), 0);
      else
        buffer.setRGB(i, 0, 0, 130 + (int)Math.round(Math.cos((timer.get()*1.25 + (i / 6)) * 2.75) * 125.0));
    }
    led.setData(buffer);
  }
}
