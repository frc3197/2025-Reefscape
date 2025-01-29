// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

/*
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
*/

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {

  private final PhotonCamera backCamera;
  private final Transform3d robotToCam = new Transform3d(new Translation3d(0.15, 0.0, 1),
      new Rotation3d(0, 0, Units.degreesToRadians(180))); // Cam mounted facing backward, half a meter forward of
                                                          // center, half a meter up from center.
  private final PhotonPoseEstimator backEstimator;

  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Drive subsystem
  private CommandSwerveDrivetrain drive;

  // private PhotonCamera intake = new PhotonCamera("USB_Camera");

  public Vision(CommandSwerveDrivetrain drive) {
    this.drive = drive;

    this.backCamera = new PhotonCamera("camera-back");
    this.backEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        robotToCam);

  }

  @Override
  public void periodic() {

    Pose3d bestPose = getDesiredPose();

    SmartDashboard.putNumber("Timestamp", getTime());

    SmartDashboard.putNumber("Limelight tag", LimelightHelpers.getFiducialID(""));

    if (bestPose != null && bestPose.getX() != 0.0) {
      SmartDashboard.putBoolean("HAS VISION", true);
      drive.addNewVisionMeasurement(bestPose.toPose2d(), getTime());

      double[] newVision = { bestPose.toPose2d().getX(), bestPose.toPose2d().getY(),
          bestPose.toPose2d().getRotation().getDegrees() };

      SmartDashboard.putNumberArray("LIMELIGHT VISION", newVision);
    } else {
      SmartDashboard.putBoolean("HAS VISION", false);
    }

    pollPhotonCameras();
  }

  // Returns the best vision pose, for updating pose estimator
  private Pose3d getDesiredPose() {
    return LimelightHelpers.getBotPose3d_wpiBlue("");
  }

  private double getTime() {
    return Timer.getFPGATimestamp();
    /*
     * return Utils.fpgaToCurrentTime(Timer.getTimestamp()
     * - (LimelightHelpers.getLatency_Pipeline(Constants.VisionConstants.
     * limelightFrontName) / 1000.0)
     * - (LimelightHelpers.getLatency_Capture(Constants.VisionConstants.
     * limelightFrontName) / 1000.0));
     */
  }

  private void pollPhotonCameras() {

    var result = backCamera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    SmartDashboard.putBoolean("hasTargets", hasTargets);

    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();

      // Get information from target
      
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

      SmartDashboard.putNumber("Back camera ambiguity", poseAmbiguity);

      drive.addNewVisionMeasurement(new Pose2d(bestCameraToTarget.getX(), bestCameraToTarget.getY(),
          bestCameraToTarget.getRotation().toRotation2d()), Timer.getFPGATimestamp());
    }
  }
}