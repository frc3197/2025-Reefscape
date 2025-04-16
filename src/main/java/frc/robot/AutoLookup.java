// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignReef;
import frc.robot.enums.AlignRequestType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Vision;

/** Add your docs here. */
@SuppressWarnings("unused")
public class AutoLookup {

        public static CommandSwerveDrivetrain drivetrain;
        private Vision vision;
        private Align align;
        private Elevator elevator;
        private Outtake outtake;
        private Algae algae;

        // 0: "FrontRight", 1: "FrontCenter", 2: "FrontLeft", 3: "BackLeft", 4:
        // "BackCenter", 5: "BackRight"

        public AutoLookup(CommandSwerveDrivetrain drive1, Vision vision1, Align align1, Elevator elevator1,
                        Outtake outtake1, Algae algae1) {
                drivetrain = drive1;
                vision = vision1;
                align = align1;
                elevator = elevator1;
                outtake = outtake1;
                algae = algae1;
        }

        public Command getLeftThreePiece() {
                return new SequentialCommandGroup(
                                drivetrain.runOnce(() -> {
                                        if (RobotContainer.isRed())
                                                drivetrain.resetPose(new Pose2d(10.481, 1.899,
                                                                new Rotation2d(Units.degreesToRadians(
                                                                                45))));
                                        else
                                                /*
                                                 * drivetrain.resetNewPose(new Pose2d(7.300, 4.180,
                                                 * new Rotation2d(Units.degreesToRadians(0))));
                                                 */
                                                drivetrain.resetPose(new Pose2d(7.049, 6.160,
                                                                new Rotation2d(Units
                                                                                .degreesToRadians(180 + 45))));
                                }),

                                // Algae arm stow
                                algae.setTargetAngleDegrees(90),

                                // Move elevator up a bit to get ready
                                elevator.setTargetHeightCommand(
                                                Constants.ElevatorConstants.level3Encoder),

                                // Go to first piece, back left R
                                loadPath("FarStartToBackLeftR"),

                                // Place first piece, back left R
                                new ParallelCommandGroup(
                                                new AlignReef(align, AlignRequestType.LEFT_REEF_ALIGN,
                                                                new ChassisSpeeds(0.9 + 0.15, 0.725 + 0.15,
                                                                                1.65 + 0.15),
                                                                new Translation3d(0.015, 0.015, 0.01),
                                                                3)
                                                                .withTimeout(1.35)
                                                                .raceWith(new WaitCommand(0.225).andThen(RobotContainer
                                                                                .getScoreSequenceL4CommandFIRST()))),

                                // Start intaking for second piece but end process after driving & delay

                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                RobotContainer.getAutoIntakeCommand().withTimeout(6.5),
                                                                elevator.setTargetHeightCommand(
                                                                                Constants.ElevatorConstants.level4Encoder
                                                                                                + 0.8),
                                                                outtake.feedOuttake(-0.235),
                                                                new WaitCommand(0.0667),
                                                                outtake.feedOuttake(0.0)),
                                                new SequentialCommandGroup(
                                                                loadPath("BackLeftRToStation"),

                                                                Commands.waitUntil(() -> {
                                                                        return vision.detectsPiece();
                                                                }).withTimeout(0.3),
                                                                new WaitCommand(0.25),

                                                                // Go to second branch, front left L
                                                                loadPath("StationToFrontLeftL"),

                                                                new AlignReef(align,
                                                                                AlignRequestType.LEFT_REEF_ALIGN,
                                                                                new ChassisSpeeds(0.9,
                                                                                                0.725,
                                                                                                1.65),
                                                                                new Translation3d(0.015,
                                                                                                0.015,
                                                                                                0.01),
                                                                                2).withTimeout(0.4))),

                                outtake.stopMotors(),
                                // RobotContainer.getIntakeCommand(),
                                elevator.setTargetHeightCommand(
                                                Constants.ElevatorConstants.level4Encoder + 1),
                                new AlignReef(align, AlignRequestType.LEFT_REEF_ALIGN,
                                                new ChassisSpeeds(0.9 + 0.15, 0.725 + 0.15, 1.65 + 0.15),
                                                new Translation3d(0.015, 0.015, 0.01),
                                                2).withTimeout(0.4),

                                // Place second piece
                                RobotContainer.getScoreSequenceL4Command(),

                                // Start intaking for third piece but end process after driving & delay
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                RobotContainer.getAutoIntakeCommand(),
                                                                elevator.setTargetHeightCommand(
                                                                                Constants.ElevatorConstants.level4Encoder
                                                                                                + 0.8),
                                                                outtake.feedOuttake(-0.2235),
                                                                new WaitCommand(0.0667),
                                                                outtake.feedOuttake(0.0)),
                                                new SequentialCommandGroup(
                                                                loadPath("FrontLeftLToStation"),

                                                                Commands.waitUntil(() -> {
                                                                        return vision.detectsPiece();
                                                                }).withTimeout(0.3),
                                                                new WaitCommand(0.25),
                                                                // Go to third branch, front left R
                                                                loadPath("StationToFrontLeftR"),
                                                                new AlignReef(align,
                                                                                AlignRequestType.RIGHT_REEF_ALIGN,
                                                                                new ChassisSpeeds(1.2,
                                                                                                1.0,
                                                                                                1.65),
                                                                                new Translation3d(0.015,
                                                                                                0.015,
                                                                                                0.01),
                                                                                2)
                                                                                .withTimeout(0.6),
                                                                Commands.runOnce(() -> {
                                                                        drivetrain.driveRobotRelative(
                                                                                        new ChassisSpeeds(
                                                                                                        0,
                                                                                                        0,
                                                                                                        0));
                                                                }))),

                                outtake.stopMotors(),

                                // Go to third branch, front left
                                elevator.setTargetHeightCommand(
                                                Constants.ElevatorConstants.level4Encoder + 0.8),

                                RobotContainer.getScoreSequenceL4Command(),
                                elevator.setTargetHeightCommand(
                                                Constants.ElevatorConstants.loadingStationEncoder),

                                // Fourth piece
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                loadPath("FrontLeftRToStation"),
                                                                Commands.waitUntil(() -> {
                                                                        return vision.detectsPiece();
                                                                }).withTimeout(0.3),
                                                                new WaitCommand(0.25),
                                                                // Go to third branch, front left R
                                                                loadPath("StationToCenter")),
                                                new SequentialCommandGroup(
                                                                RobotContainer.getAutoIntakeCommand(),
                                                                elevator.setTargetHeightCommand(
                                                                                Constants.ElevatorConstants.level2Encoder),
                                                                outtake.feedOuttake(-0.2),
                                                                new WaitCommand(0.0667),
                                                                outtake.feedOuttake(0.0))),
                                outtake.feedOuttake(0.4)

                );
        }

        public Command getCenterLeftHighNetNet() {
                return new SequentialCommandGroup(
                                Commands.runOnce(() -> {
                                        if (RobotContainer.isRed()) {
                                                drivetrain.resetPose(new Pose2d(10.250, 4.180,
                                                                new Rotation2d(Units.degreesToRadians(0))));
                                        } else {
                                                drivetrain.resetPose(new Pose2d(7.300, 4.180,
                                                                new Rotation2d(Units.degreesToRadians(180))));
                                        }
                                }),
                                // TODO FINISH THIS IT DOESN'T WORK YET

                                // Algae arm stow
                                algae.setTargetAngleDegrees(90),

                                // Move elevator up a bit to get ready
                                elevator.setTargetHeightCommand(
                                                Constants.ElevatorConstants.level3Encoder),

                                // Go to first piece, back left R
                                loadPath("CenterToLeftCenterBack"),

                                // Place first piece, back left R
                                RobotContainer.getScoreSequenceL4Command(),
                                elevator.setTargetHeightCommand(Constants.ElevatorConstants.elevatorLowHeight),
                                new ParallelCommandGroup(
                                                loadPath("CenterAndBack"),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.385),
                                                                RobotContainer.getAlgaeLowCommandAUTOFIRST())),

                                new ParallelCommandGroup(loadPath("CenterToNet"),
                                                new SequentialCommandGroup(RobotContainer.getAlgaeCommandAUTOSECOND(),
                                                                RobotContainer.getAlgaeScoreNetInitial())),
                                RobotContainer.getAlgaeScoreNetFinal(),
                                new ParallelCommandGroup(
                                                RobotContainer.getAlgaeHighCommandALGAEFIRST(),
                                                loadPath("NetToBackLeft")),
                                new ParallelCommandGroup(loadPath("LeftToNet"),
                                                new SequentialCommandGroup(RobotContainer.getAlgaeCommandAUTOSECOND(),
                                                                RobotContainer.getAlgaeScoreNetInitial())),
                                RobotContainer.getAlgaeScoreNetFinal(),
                                loadPath("NetOffLine")

                );
        }

        public Command getAuto(String name) {
                if (name.equals("Left 3")) {
                        return getLeftThreePiece();
                }
                if (name.equals("Center-L High-Net-Net")) {
                        return getCenterLeftHighNetNet();
                }
                // System.out.println(name);
                return new InstantCommand();
        }

        public Command getTestAuto() {
                return new SequentialCommandGroup(Commands.runOnce(() -> {
                        if (RobotContainer.isRed()) {
                                drivetrain.resetPose(new Pose2d(10.250, 4.180,
                                                new Rotation2d(Units.degreesToRadians(0))));
                        } else {
                                drivetrain.resetPose(new Pose2d(7.300, 4.180,
                                                new Rotation2d(Units.degreesToRadians(180))));
                        }
                }),
                                elevator.getEncoderResetCommand(),
                                algae.setTargetAngleDegrees(90),
                                loadPath("NewAuto"));
        }

        private Command loadPath(String name) {
                try {
                        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
                } catch (FileVersionException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                } catch (IOException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                } catch (ParseException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                }
                return null;
        }
}
