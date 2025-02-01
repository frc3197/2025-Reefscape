// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.AlignRequestType;
import frc.robot.enums.RobotMode;

@SuppressWarnings("unused")
public class Align extends SubsystemBase {

  private int requestedAlignSection = 1;
  private final CommandSwerveDrivetrain drive;

  public Align(CommandSwerveDrivetrain drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("align_section", getSection());
    SmartDashboard.putNumber("controller angle", RobotContainer.getAlignRequestAngle());
  }

  // Returns selected section, adds rumble if changed
  public int getSection() {
    double oldSection = requestedAlignSection;

    if (RobotContainer.getAlignRequestDistance()) {
      if (isBetween(RobotContainer.getAlignRequestAngle(), 0, 30)
          || isBetween(RobotContainer.getAlignRequestAngle(), 330, 360)) {
        requestedAlignSection = 4;
      } else if (isBetween(RobotContainer.getAlignRequestAngle(), 30, 90)) {
        requestedAlignSection = 5;
      } else if (isBetween(RobotContainer.getAlignRequestAngle(), 90, 150)) {
        requestedAlignSection = 0;
      } else if (isBetween(RobotContainer.getAlignRequestAngle(), 150, 210)) {
        requestedAlignSection = 1;
      } else if (isBetween(RobotContainer.getAlignRequestAngle(), 210, 270)) {
        requestedAlignSection = 2;
      } else if (isBetween(RobotContainer.getAlignRequestAngle(), 270, 330)) {
        requestedAlignSection = 3;
      }
    }
    if (oldSection != requestedAlignSection) {
      RobotContainer.addRumble(1, 0.35, 0.15, RumbleType.kBothRumble);
    }

    return requestedAlignSection;
  }

  // Helper function to check if angle is between other angles
  private boolean isBetween(double value, double lower, double upper) {
    return value >= lower && value <= upper;
  }

  // Align reef command
  public Command alignReefRough(AlignRequestType side) {

    // ReefAlignment/
    String pathName = "";

    switch (requestedAlignSection) {
      case 0:
        pathName = pathName.concat("FrontRight");
        break;
      case 1:
        pathName = pathName.concat("FrontCenter");
        break;
      case 2:
        pathName = pathName.concat("FrontLeft");
        break;
      case 3:
        pathName = pathName.concat("BackLeft");
        break;
      case 4:
        pathName = pathName.concat("BackCenter");
        break;

      default:
        break;
    }

    if (side == AlignRequestType.LEFT_REEF_ALIGN)
      return Commands.runOnce(() -> {RobotContainer.setRobotMode(RobotMode.ALIGN_REEF_ROUGH);}).andThen(drive.alignReef(pathName.concat("L"))).andThen(Commands.runOnce(() -> {RobotContainer.setRobotMode(RobotMode.NONE);}));
      else
      return Commands.runOnce(() -> {RobotContainer.setRobotMode(RobotMode.ALIGN_REEF_ROUGH);}).andThen(drive.alignReef(pathName.concat("R"))).andThen(Commands.runOnce(() -> {RobotContainer.setRobotMode(RobotMode.NONE);}));
  }
}
