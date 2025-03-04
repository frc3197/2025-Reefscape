// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
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

  private int requestedAlignSection = 2;
  private final CommandSwerveDrivetrain drive;

  private int timesAligned = 0;

  private Command leftAlignCommand = null;
  private Command rightAlignCommand = null;

  private final SwerveRequest.FieldCentric swerveRequest;

  private ChassisSpeeds speeds = new ChassisSpeeds();

  private PIDController xController = Constants.AlignConstants.xController;
  private PIDController yController = Constants.AlignConstants.yController;
  private PIDController thetaController = Constants.AlignConstants.thetaController;

  private PIDController xControllerRobot = Constants.AlignConstants.xController;
  private PIDController yControllerRobot = Constants.AlignConstants.yController;
  private PIDController thetaControllerRobot = Constants.AlignConstants.thetaController;

  private double xError = 0.05;
  private double yError = 0.05;
  private double thetaError = 0.1;

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

    Pose2d leftTargetPose;
    Pose2d rightTargetPose;

    if (RobotContainer.isRed()) {
      leftTargetPose = Constants.AlignPositions.RedPositions.redFeefPoses[requestedAlignSection][0];
      rightTargetPose = Constants.AlignPositions.RedPositions.redFeefPoses[requestedAlignSection][1];
    } else {
      leftTargetPose = Constants.AlignPositions.BluePositions.blueFeefPoses[requestedAlignSection][0];
      rightTargetPose = Constants.AlignPositions.BluePositions.blueFeefPoses[requestedAlignSection][1];
    }

    double[] leftPoseArray = new double[3];
    double[] rightPoseArray = new double[3];

    leftPoseArray[0] = leftTargetPose.getX();
    leftPoseArray[1] = leftTargetPose.getY();
    leftPoseArray[2] = leftTargetPose.getRotation().getDegrees();

    rightPoseArray[0] = rightTargetPose.getX();
    rightPoseArray[1] = rightTargetPose.getY();
    rightPoseArray[2] = rightTargetPose.getRotation().getDegrees();

    SmartDashboard.putNumberArray("LeftTargetPose", leftPoseArray);
    SmartDashboard.putNumberArray("RightTargetPose", rightPoseArray);

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
      // setAlignCommands();
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
    // leftAlignCommand = drive.alignReef(pathName.concat("L"));
    // rightAlignCommand = drive.alignReef(pathName.concat("R"));
  }

  public Command alignBarge() {
    return drive.pathfindToPose(
        RobotContainer.isRed() ? Constants.AlgaeConstants.redBargePose : Constants.AlgaeConstants.blueBargePose);
  }

  // Helper function to check if angle is between other angles
  private boolean isBetween(double value, double lower, double upper) {
    return value >= lower && value <= upper;
  }

  public boolean runRobotRelativeAlign(AlignRequestType request, ChassisSpeeds maximumSpeeds, Translation3d goalErrors,
      int section) {

    if (section != -1) {
      requestedAlignSection = section;
    }

    int side = 0;
    if (request == AlignRequestType.RIGHT_REEF_ALIGN) {
      side = 1;
    }

    double maxSpeedX = Math.abs(maximumSpeeds.vxMetersPerSecond);
    double maxSpeedY = Math.abs(maximumSpeeds.vyMetersPerSecond);
    double maxSpeedTheta = Math.abs(maximumSpeeds.omegaRadiansPerSecond);

    Pose2d targetPose;

    if (RobotContainer.isRed()) {
      targetPose = Constants.AlignPositions.RedPositions.redFeefPoses[requestedAlignSection][side];
    } else {
      targetPose = Constants.AlignPositions.BluePositions.blueFeefPoses[requestedAlignSection][side];
    }

    Pose2d pose = drive.getNewCurrentPose();

    double xAlignSpeed = MathUtil.clamp(xController.calculate(pose.getX(), targetPose.getX()), -maxSpeedX, maxSpeedX);
    double yAlignSpeed = MathUtil.clamp(yController.calculate(pose.getY(), targetPose.getY()), -maxSpeedY, maxSpeedY);
    double thetaAlignSpeed = MathUtil.clamp(
        thetaController.calculate(drive.getNewCurrentPose().getRotation().getRadians(),
            targetPose.getRotation().getRadians()),
        -maxSpeedTheta, maxSpeedTheta);

    if (Math.abs(pose.getX() - targetPose.getX()) < goalErrors.getX()) {
      xAlignSpeed = 0.0;
    }

    if (Math.abs(pose.getY() - targetPose.getY()) < goalErrors.getY()) {
      yAlignSpeed = 0.0;
    }

    if (Math
        .abs(drive.getNewCurrentPose().getRotation().getRadians() - targetPose.getRotation().getRadians()) < goalErrors
            .getZ()) {
      thetaAlignSpeed = 0.0;
    }

    speeds = new ChassisSpeeds(xAlignSpeed, yAlignSpeed, thetaAlignSpeed);

    drive.driveFieldRelative(speeds);

    return xAlignSpeed == 0.0 && yAlignSpeed == 0.0 && thetaAlignSpeed == 0.0;
  }

  public boolean runCustomAlign(AlignRequestType request, ChassisSpeeds maximumSpeeds, Translation3d goalErrors,
      int section) {

    if (section != -1) {
      requestedAlignSection = section;
    }

    int side = 0;
    if (request == AlignRequestType.RIGHT_REEF_ALIGN) {
      side = 1;
    }

    double maxSpeedX = Math.abs(maximumSpeeds.vxMetersPerSecond);
    double maxSpeedY = Math.abs(maximumSpeeds.vyMetersPerSecond);
    double maxSpeedTheta = Math.abs(maximumSpeeds.omegaRadiansPerSecond);

    Pose2d targetPose;

    if (RobotContainer.isRed()) {
      targetPose = Constants.AlignPositions.RedPositions.redFeefPoses[requestedAlignSection][side];
    } else {
      targetPose = Constants.AlignPositions.BluePositions.blueFeefPoses[requestedAlignSection][side];
    }

    Pose2d pose = drive.getNewCurrentPose();

    double xAlignSpeed = MathUtil.clamp(xController.calculate(pose.getX(), targetPose.getX()), -maxSpeedX, maxSpeedX);
    double yAlignSpeed = MathUtil.clamp(yController.calculate(pose.getY(), targetPose.getY()), -maxSpeedY, maxSpeedY);
    double thetaAlignSpeed = MathUtil.clamp(
        thetaController.calculate(drive.getNewCurrentPose().getRotation().getRadians(),
            targetPose.getRotation().getRadians()),
        -maxSpeedTheta, maxSpeedTheta);

    if (Math.abs(pose.getX() - targetPose.getX()) < goalErrors.getX()) {
      xAlignSpeed = 0.0;
    }

    if (Math.abs(pose.getY() - targetPose.getY()) < goalErrors.getY()) {
      yAlignSpeed = 0.0;
    }

    if (Math
        .abs(drive.getNewCurrentPose().getRotation().getRadians() - targetPose.getRotation().getRadians()) < goalErrors
            .getZ()) {
      thetaAlignSpeed = 0.0;
    }

    speeds = new ChassisSpeeds(xAlignSpeed, yAlignSpeed, thetaAlignSpeed);

    drive.driveFieldRelative(speeds);

    return xAlignSpeed == 0.0 && yAlignSpeed == 0.0 && thetaAlignSpeed == 0.0;
  }

  public void alignReefRough(AlignRequestType side) {

    if (side == AlignRequestType.LEFT_REEF_ALIGN)
      CommandScheduler.getInstance().schedule(leftAlignCommand.andThen(alignReefColor()));
    else
      CommandScheduler.getInstance().schedule(rightAlignCommand.andThen(alignReefColor()));

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

  public Command alignReefColor() {
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
