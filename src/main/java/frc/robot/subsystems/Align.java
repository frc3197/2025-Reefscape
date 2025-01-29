// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.AlignRequestType;

public class Align extends SubsystemBase {

  private int requestedAlignSection = 1;

  private CommandSwerveDrivetrain drive;

  private Supplier<Integer> sectionSupplier;

  private Command leftAlignCommand;
  private Command rightAlignCommand;

  public Align(CommandSwerveDrivetrain drive) {
    this.drive = drive;

    sectionSupplier = this::getSection;

    setAlignCommands();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("align_section", getSection());
    SmartDashboard.putNumber("controller angle", RobotContainer.getAlignRequestAngle());
  }

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
      setAlignCommands();
    }

    return requestedAlignSection;
  }

  private boolean isBetween(double value, double lower, double upper) {
    return value >= lower && value <= upper;
  }

  private void setAlignCommands() {
    String pathName = "";

    switch (requestedAlignSection) {
      case 0:
        pathName = "FrontRight";
        break;
      case 1:
        pathName = "FrontCenter";
        break;
      case 2:
        pathName = "FrontLeft";
        break;
      case 3:
        pathName = "BackLeft";
        break;
      case 4:
        pathName = "BackCenter";
        break;

      default:
        break;
    }

    System.out.println(pathName);
    System.out.println(requestedAlignSection);

    leftAlignCommand = drive.alignReef(pathName.concat("L"));
    rightAlignCommand = drive.alignReef(pathName.concat("R"));
  }

  public void alignReef(AlignRequestType side) {
    if (side == AlignRequestType.LeftReefAlign)
      CommandScheduler.getInstance().schedule(leftAlignCommand);
    else
      CommandScheduler.getInstance().schedule(rightAlignCommand);
  }
}
