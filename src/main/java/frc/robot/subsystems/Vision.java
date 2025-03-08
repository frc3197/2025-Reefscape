// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.RobotMode;
import frc.robot.util.LimelightHelpers;

@SuppressWarnings("unused")
public class Vision extends SubsystemBase {

  // Photonvision variables
  private final PhotonCamera backCamera;
  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;
  private final Transform3d robotToBack = new Transform3d(new Translation3d(0.15, 0.0, 1),
      new Rotation3d(0, 0, Units.degreesToRadians(180))); // Cam mounted facing backward, half a meter forward of
                                                          // center, half a meter up from center.

  private final Transform3d robotToLeft = new Transform3d(
      new Translation3d(Units.inchesToMeters(-2.75), Units.inchesToMeters(-11), Units.inchesToMeters(7.25)),
      new Rotation3d(0, Units.degreesToRadians(-63 + 180), Units.degreesToRadians(-6)));

  private final Transform3d robotToRight = new Transform3d(
      new Translation3d(Units.inchesToMeters(-9.75), Units.inchesToMeters(-11), Units.inchesToMeters(7.25)),
      new Rotation3d(0, Units.degreesToRadians(-63), Units.degreesToRadians(6)));
  private final PhotonPoseEstimator backEstimator;
  private final PhotonPoseEstimator leftEstimator;
  private final PhotonPoseEstimator rightEstimator;
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Reef & algae align camera
  private final PhotonCamera alignCamera;
  private final PhotonCamera intakeCamera;

  // Drive subsystem
  private CommandSwerveDrivetrain drive;

  public Vision(CommandSwerveDrivetrain drive) {
    this.drive = drive;

    this.backCamera = new PhotonCamera("camera-back");
    this.leftCamera = new PhotonCamera("camera-left");
    this.rightCamera = new PhotonCamera("camera-right");
    this.intakeCamera = new PhotonCamera("intake-camera");
    this.backEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        robotToBack);
    this.leftEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        robotToLeft);
    this.rightEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        robotToRight);

    this.alignCamera = new PhotonCamera("camera-align");
  }

  @Override
  public void periodic() {

    Pose3d bestPose = getDesiredPose();

    SmartDashboard.putNumber("Timestamp", getLimelightTime());
    SmartDashboard.putNumber("Limelight tag", LimelightHelpers.getFiducialID(""));

    if (bestPose != null && bestPose.getX() != 0.0) {
      SmartDashboard.putBoolean("HAS VISION", true);
      drive.addNewVisionMeasurement(bestPose.toPose2d(), getLimelightTime());

      double[] newVision = { bestPose.toPose2d().getX(), bestPose.toPose2d().getY(),
          bestPose.toPose2d().getRotation().getDegrees() };

      SmartDashboard.putNumberArray("LIMELIGHT VISION", newVision);
    } else {
      SmartDashboard.putBoolean("HAS VISION", false);
    }

    pollPhotonCameras();

    checkIntakeCamera();
  }

  // Returns the best vision pose, for updating pose estimator
  private Pose3d getDesiredPose() {
    return LimelightHelpers.getBotPose3d_wpiBlue("");
  }

  // Returns time, needs to be fixed
  private double getLimelightTime() {
    return Timer.getTimestamp();

    /*
     * return Utils.fpgaToCurrentTime(Timer.getTimestamp()
     * - (LimelightHelpers.getLatency_Pipeline("") / 1000.0)
     * - (LimelightHelpers.getLatency_Capture("") / 1000.0));
     */

  }

  // Check photon camera outputs
  private void pollPhotonCameras() {

    var backResult = backCamera.getLatestResult();
    boolean backHasTargets = backResult.hasTargets();

    var leftResult = leftCamera.getLatestResult();
    boolean leftHasTargets = leftResult.hasTargets();

    var rightResult = rightCamera.getLatestResult();
    boolean rightHasTargets = rightResult.hasTargets();

    SmartDashboard.putBoolean("hasTargets", backHasTargets);

    if (backHasTargets && false) {
      PhotonTrackedTarget target = backResult.getBestTarget();

      // Get information from target
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

      drive.addNewVisionMeasurement(new Pose2d(bestCameraToTarget.getX(), bestCameraToTarget.getY(),
          bestCameraToTarget.getRotation().toRotation2d()), Timer.getFPGATimestamp());
    }

    if (leftHasTargets) {
      PhotonTrackedTarget target = leftResult.getBestTarget();

      // Get information from target
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();

      Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
          aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotToLeft);
      Transform3d robotOffset = target.getBestCameraToTarget();

      double[] newVision = { robotPose.toPose2d().getX(), robotPose.toPose2d().getY(),
        robotPose.toPose2d().getRotation().getDegrees() };

      SmartDashboard.putNumberArray("LEFT CAMERA VISION", newVision);

      if (Math.sqrt(
          Math.pow(robotOffset.getX(), 2) + Math.pow(robotOffset.getY(), 2) + Math.pow(robotOffset.getZ(), 2)) < 2)
        drive.addNewVisionMeasurement(robotPose.toPose2d(), leftResult.getTimestampSeconds());
    }

    if (rightHasTargets) {
      PhotonTrackedTarget target = rightResult.getBestTarget();

      // Get information from target
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();

      Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
          aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotToRight);

      Transform3d robotOffset = target.getBestCameraToTarget();

      double[] newVision = { robotPose.toPose2d().getX(), robotPose.toPose2d().getY(),
        robotPose.toPose2d().getRotation().getDegrees() };

      SmartDashboard.putNumberArray("RIGHT CAMERA VISION", newVision);

      if (Math.sqrt(
          Math.pow(robotOffset.getX(), 2) + Math.pow(robotOffset.getY(), 2) + Math.pow(robotOffset.getZ(), 2)) < 2)
        drive.addNewVisionMeasurement(robotPose.toPose2d(), rightResult.getTimestampSeconds());
    }
  }

  public double[] getBestAlignCameraTarget() {
    var result = alignCamera.getLatestResult();
    if (result.hasTargets()) {
      var bestTarget = result.getBestTarget();
      return new double[] { bestTarget.getYaw(), bestTarget.getPitch() };
    }
    return new double[] { 0, 0 };
  }

  public void checkIntakeCamera() {
    var result = intakeCamera.getLatestResult();
    if (result.hasTargets()) {
      var bestTarget = result.getBestTarget();
      RobotContainer.setRobotMode(RobotMode.DETECTS_PIECE);
      return;
    }
    if(RobotContainer.getRobotMode() == RobotMode.DETECTS_PIECE) {
      RobotContainer.setRobotMode(RobotMode.NONE);
    }
  }

  public boolean detectsPiece() {
    var result = intakeCamera.getLatestResult();
    if (result.hasTargets()) {
      var bestTarget = result.getBestTarget();
      RobotContainer.setRobotMode(RobotMode.DETECTS_PIECE);
      return true;
    }
    return false;
  }
}