// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Align extends SubsystemBase {

  public Align() {
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("align_section", getSection());
  }

  private int getSection() {
    return 0;
  }
}
