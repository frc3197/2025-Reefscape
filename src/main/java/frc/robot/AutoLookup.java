// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
                // Set initial position
                Commands.runOnce(() -> {
                    drivetrain.resetNewPose(new Pose2d(10.297, 0.465, new Rotation2d(0)));
                }),
                // Algae arm stow
                algae.setTargetAngleDegrees(65),

                // To reef first piece
                loadPath("FarStartToL4"),

                align.alignReefRoughWithString("FrontLeftL"),
                // align.alignReefFine(),

                elevator.setTargetHeightCommand(Constants.ElevatorConstants.alignIdleEncoder),

                Commands.waitUntil(() -> {return elevator.getPositionError() < 100;}),

                align.alignReefFine(),

                new InstantCommand(() -> drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0))),

                RobotContainer.getScoreSequenceL4Command(),

                loadPath("L4ToStation"),
                RobotContainer.getIntakeCommand(),
                
                loadPath("StationToL4"),
                align.alignReefRoughWithString("FrontLeftR"),
                // align.alignReefFine(),
                
                elevator.setTargetHeightCommand(Constants.ElevatorConstants.alignIdleEncoder),
                
                Commands.waitUntil(() -> {return elevator.getPositionError() < 100;}),
                
                align.alignReefFine(),
                
                new InstantCommand(() -> drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0))),
                
                RobotContainer.getScoreSequenceL4Command(),
                loadPath("L4ToStation")
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
