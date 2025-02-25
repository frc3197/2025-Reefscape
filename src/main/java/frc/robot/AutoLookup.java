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

    public Command getAuto() {
        return new SequentialCommandGroup(
                // Set initial position nope lol

                // Algae arm stow
                algae.setTargetAngleDegrees(90),

                // Move elevator up a bit to get ready
                elevator.setTargetHeightCommand(Constants.ElevatorConstants.level3Encoder + 5000),

                // Go to first piece, back left R
                loadPath("FarStartToBackLeftR"),

                // Place first piece, back left R
                new ParallelCommandGroup(
                        new AlignReef(align, AlignRequestType.RIGHT_REEF_ALIGN, new ChassisSpeeds(0.9, 0.725, 1.65),
                                new Translation3d(0.015, 0.015, 0.01), 3).withTimeout(0.5),
                        RobotContainer.getScoreSequenceL4Command()),

                // Start intaking for second piece but end process after driving & delay
                /*
                 * new SequentialCommandGroup(RobotContainer.getIntakeCommand()).raceWith(
                 * new SequentialCommandGroup(
                 * new WaitCommand(0.45))),
                 */
                loadPath("BackLeftRToStation"),
                // Commands.runOnce(() -> {drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0,
                // 0));}),

                new WaitCommand(0.415),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                RobotContainer.getAutoIntakeCommand(),
                                outtake.feedOuttake(-0.2),
                                new WaitCommand(0.0667),
                                outtake.feedOuttake(0.0),
                                elevator.setTargetHeightCommand(Constants.ElevatorConstants.level2Encoder)),
                        new SequentialCommandGroup(

                                // Go to second branch, front left L
                                loadPath("StationToFrontLeftL"),
                                // Commands.runOnce(() -> {
                                // drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0,
                                // 0));
                                // }))),
                                new AlignReef(align, AlignRequestType.LEFT_REEF_ALIGN,
                                        new ChassisSpeeds(0.9, 0.725, 1.65),
                                        new Translation3d(0.015, 0.015, 0.01), 2).withTimeout(1.2))),

                outtake.stopMotors(),
                // RobotContainer.getIntakeCommand(),
                elevator.setTargetHeightCommand(Constants.ElevatorConstants.level3Encoder + 2500),

                // Place second piece
                RobotContainer.getScoreSequenceL4Command(),

                // Start intaking for third piece but end process after driving & delay
                /*
                 * RobotContainer.getIntakeCommand().raceWith(
                 * new SequentialCommandGroup(
                 * new WaitCommand(0.45))),
                 */
                loadPath("FrontLeftLToStation"),
                // Commands.runOnce(() -> {drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0,
                // 0));}),
                new WaitCommand(0.375),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                RobotContainer.getAutoIntakeCommand(),
                                elevator.setTargetHeightCommand(Constants.ElevatorConstants.level3Encoder + 10000),
                                outtake.feedOuttake(-0.2),
                                new WaitCommand(0.0667),
                                outtake.feedOuttake(0.0)),
                        new SequentialCommandGroup(

                                // Go to third branch, front left R
                                loadPath("StationToFrontLeftR"),
                                Commands.runOnce(() -> {
                                    drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0,
                                            0));
                                }))),

                outtake.stopMotors(),

                // Go to third branch, front left
                elevator.setTargetHeightCommand(Constants.ElevatorConstants.level3Encoder + 10000),

                // Make sure piece is intake and align finely
                new ParallelCommandGroup(
                        // RobotContainer.getIntakeCommand(),
                        new AlignReef(align, AlignRequestType.RIGHT_REEF_ALIGN, new ChassisSpeeds(1.2, 1.0, 1.65),
                                new Translation3d(0.015, 0.015, 0.01), 2).withTimeout(1)),

                RobotContainer.getScoreSequenceL4Command()

        );
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
