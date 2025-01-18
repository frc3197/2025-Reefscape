// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.enums.AlignRequestType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    
                                                                                      // max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Vision poseEstimator = new Vision(drivetrain);
    private final Align align = new Align(drivetrain);
    private final Light light = new Light();

    private final static CommandXboxController driverController = new CommandXboxController(0);
    private final static CommandXboxController operatorController = new CommandXboxController(1);

    private final static CommandXboxController[] controllers = { driverController, operatorController };

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * TunerConstants.MaxSpeed) // Drive
                                                                                                           // forward
                                                                                                           // with
                        // negative Y
                        // (forward)
                        .withVelocityY(-driverController.getLeftX() * TunerConstants.MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverController.getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise
                                                                                            // with
                // negative X (left)
                ));

                /* 
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(
                        new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));
                        */

        driverController.leftTrigger().onTrue(align.alignReef(AlignRequestType.LeftAlign));
        driverController.rightTrigger().onTrue(align.alignReef(AlignRequestType.RightAlign));

        /* 
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // reset the field-centric heading on left bumper press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public static double getAlignRequestAngle() {
        if (operatorController.getLeftX() > 0) {
            return ((Math.atan(operatorController.getLeftY() / operatorController.getLeftX()) + (Math.PI / 2))
                    / (Math.PI * 2)) * 360;
        } else {
            return (((Math.atan(operatorController.getLeftY() / operatorController.getLeftX()) + (Math.PI / 2))
                    / (Math.PI * 2)) * 360) + 180;
        }
    }

    public static boolean getAlignRequestDistance() {
        return Math.sqrt(Math.pow(operatorController.getLeftX(), 2) + Math.pow(operatorController.getLeftY(), 2)) > 0.8;
    }

    public static void addRumble(int controller, double value, double time, RumbleType type) {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            controllers[controller].setRumble(type, value);
                        }),
                        new WaitCommand(time),
                        new InstantCommand(() -> {
                            controllers[controller].setRumble(type, 0);
                        })));
    }

    public static boolean isRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    private Command runAuto = drivetrain.getAutoPath("Test");

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
                runAuto);
    }
}
