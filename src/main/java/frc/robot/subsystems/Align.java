// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.AlignRequestType;
import frc.robot.RobotContainer;

public class Align extends SubsystemBase {

  private int requestedAlignSection = 0;

  private CommandSwerveDrivetrain drive;

  private Pose2d[] redReefPoses = {new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 1, new Rotation2d(0)), new Pose2d(2, 2, new Rotation2d(0)), new Pose2d(3, 3, new Rotation2d(0)), new Pose2d(4, 4, new Rotation2d(0)), new Pose2d(5, 5, new Rotation2d(0))};

  public Align(CommandSwerveDrivetrain drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("align_section", getSection());
    SmartDashboard.putNumber("controller angle", RobotContainer.getAlignRequestAngle());
  }

  private int getSection() {
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

  private boolean isBetween(double value, double lower, double upper) {
    return value >= lower && value <= upper;
  }

  public Command alignReef(AlignRequestType side) {
    return drive.pathFind(Constants.RedAlignPositions[0][side.ordinal()]);
  }
}
