// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.AlertMode;
import frc.robot.enums.AlignRequestType;
import frc.robot.enums.RobotMode;
import frc.robot.util.AlertBody;

@SuppressWarnings("unused")
public class Align extends SubsystemBase {

  private int requestedAlignSection = 1;
  private final CommandSwerveDrivetrain drive;

  private int timesAligned = 0;

  private Command leftAlignCommand = null;
  private Command rightAlignCommand = null;

  private final SwerveRequest.FieldCentric swerveRequest;

  private ChassisSpeeds speeds = new ChassisSpeeds();

  private int alignUpdateSpeedTicker = 0;

  public Align(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    this.swerveRequest = new SwerveRequest.FieldCentric();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("align_section", getSection());
    SmartDashboard.putNumber("controller angle", RobotContainer.getAlignRequestAngle());
    setAlignCommands();

    if (RobotContainer.getBestAlignCameraTarget()[1] < 0.5) {
      timesAligned++;
    }

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
      setAlignCommands();
    }

    return requestedAlignSection;
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
    leftAlignCommand = drive.alignReef(pathName.concat("L"));
    rightAlignCommand = drive.alignReef(pathName.concat("R"));
  }

  // Helper function to check if angle is between other angles
  private boolean isBetween(double value, double lower, double upper) {
    return value >= lower && value <= upper;
  }

  public void runCustomAlign(AlignRequestType request) {
    int side = 0;
    if (request == AlignRequestType.RIGHT_REEF_ALIGN) {
      side = 1;
    }

    Pose2d targetPose = Constants.AlignPositions.RedPositions.redFeefPoses[requestedAlignSection][side];
    Pose2d direction = drive.getNewCurrentPose().relativeTo(targetPose);

    if (alignUpdateSpeedTicker >= 7 || true) {
      speeds = new ChassisSpeeds(-MathUtil.clamp(direction.getX() * 3, -1, 1),
          -MathUtil.clamp(direction.getY() * 3, -2, 2),
          MathUtil.clamp(direction.getRotation().getRadians() / 4, -0.3, 0.3));
          alignUpdateSpeedTicker = 0;
    }
    alignUpdateSpeedTicker ++;

    drive.driveFieldRelative(speeds);

  }

  public void alignReefRough(AlignRequestType side) {

    if (side == AlignRequestType.LEFT_REEF_ALIGN)
      CommandScheduler.getInstance().schedule(leftAlignCommand.andThen(alignReefFine()));
    else
      CommandScheduler.getInstance().schedule(rightAlignCommand.andThen(alignReefFine()));
  }

  // Align reef command
  public Command alignReefRoughWithString(String path) {
    return Commands.runOnce(() -> {
      RobotContainer.setRobotMode(RobotMode.ALIGN_REEF_ROUGH);
    }).andThen(new SequentialCommandGroup(
        drive.alignReef(path),
        Commands.runOnce(() -> {
          RobotContainer.setRobotMode(RobotMode.NONE);
        })));
  };

  public Command alignReefFine() {
    return Commands.runOnce(() -> {
      RobotContainer.setRobotMode(RobotMode.ALIGN_REEF_FINE);
    }).andThen(Commands.run(() -> {
      timesAligned = 0;
      double[] offsets = RobotContainer.getBestAlignCameraTarget();
      double strafeSpeed = Constants.VisionConstants.strafeAlignPID.calculate(offsets[0]);
      strafeSpeed = MathUtil.clamp(strafeSpeed, -1, 1);
      ChassisSpeeds speeds = new ChassisSpeeds(0, strafeSpeed, 0);

      drive.driveRobotRelative(speeds);
    }).until(() -> {
      return Math.abs(RobotContainer.getBestAlignCameraTarget()[1]) < 0.5 && timesAligned > 5;
    })).withTimeout(1.15).andThen(Commands.runOnce(() -> {
      RobotContainer.setRobotMode(RobotMode.NONE);
      RobotContainer.addAlert(new AlertBody(AlertMode.FULLY_ALIGNED, 0.8));
    }));
  }
}
