// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;

/*
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
*/

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Vision extends SubsystemBase {

  // Drive subsystem
  private CommandSwerveDrivetrain drive;

  // Limelight
  private NetworkTable limelightNetworkTable;
  private double cameraLatency;

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("NewPose", Pose2d.struct).publish();

  // private PhotonCamera intake = new PhotonCamera("USB_Camera");

  public Vision(CommandSwerveDrivetrain drive) {
    this.drive = drive;

    limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight-front");
  }

  @Override
  public void periodic() {

    Pose3d bestPose = getDesiredPose();

    if (bestPose != null) {
      drive.addNewVisionMeasurement(bestPose.toPose2d(), Timer.getFPGATimestamp() - cameraLatency);
    }
    // poseEstimator.addVisionMeasurement(new Pose2d(5, 5, new Rotation2d(0)),
    // Timer.getFPGATimestamp() - cameraLatency);

    publisher.set(drive.getNewCurrentPose());

  }

  // Returns the best vision pose, for updating pose estimator
  private Pose3d getDesiredPose() {

    // If limelight tag has enough area, favor it over side cameras
    if (getLimelightArea() > 0) {
      double[] limelightBotPose = getBotPoseBlue();

      cameraLatency = limelightBotPose[6] / 1000.0;

      return new Pose3d(limelightBotPose[0], limelightBotPose[1], limelightBotPose[2],
          new Rotation3d(Units.degreesToRadians(limelightBotPose[3]), Units.degreesToRadians(limelightBotPose[4]),
              Units.degreesToRadians(limelightBotPose[5])));
    }
    return null;
  }

  // Returns limelight tag area
  private double getLimelightArea() {
    return limelightNetworkTable.getEntry("ta").getDouble(0);
  }

  // Returns limelight pose relative to blue origin
  private double[] getBotPoseBlue() {
    return limelightNetworkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
  }

  public void setLimelightLight(int mode) {
    limelightNetworkTable.getEntry("ledMode").setNumber(mode);
  }
}