// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

/*
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
*/

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {

  // Drive subsystem
  private CommandSwerveDrivetrain drive;

  // private PhotonCamera intake = new PhotonCamera("USB_Camera");

  public Vision(CommandSwerveDrivetrain drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {

    Pose3d bestPose = getDesiredPose();

    if (bestPose != null) {
      drive.addNewVisionMeasurement(bestPose.toPose2d(), getTime());
    }
  }

  // Returns the best vision pose, for updating pose estimator
  private Pose3d getDesiredPose() {
    return LimelightHelpers.getBotPose3d_wpiBlue(Constants.VisionConstants.limelightFrontName);
  }

  private double getTime() {
    return Utils.fpgaToCurrentTime(Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline(Constants.VisionConstants.limelightFrontName)/1000.0) - (LimelightHelpers.getLatency_Capture(Constants.VisionConstants.limelightFrontName)/1000.0));
  }
}