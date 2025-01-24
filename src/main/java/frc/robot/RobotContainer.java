// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.enums.LightPattern;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.enums.*;
import frc.robot.managersubsystems.LightManager;
import frc.robot.managersubsystems.ErrorManager;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Vision;
import frc.robot.util.*;

@SuppressWarnings("unused")
public class RobotContainer {

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add
                                                                                                                       // a
                                                                                                                       // 10%
                                                                                                                       // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);

    // Managers
    private final static LightManager lightManager = new LightManager();
    private final static ErrorManager errorManager = new ErrorManager();

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Vision poseEstimator = new Vision(drivetrain);
    private final Align align = new Align(drivetrain);
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Outtake outtake = new Outtake();
    private final Algae algae = new Algae();

    // Controllers
    private final static CommandXboxController driverController = new CommandXboxController(0);
    private final static CommandXboxController operatorController = new CommandXboxController(1);

    private final static CommandXboxController[] controllers = { driverController, operatorController };

    public static boolean isEnabled = false;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain
                        .applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * TunerConstants.MaxSpeed) // Drive
                                // forward
                                // with
                                // negative Y
                                // (forward)
                                .withVelocityY(-driverController.getLeftX() * TunerConstants.MaxSpeed) // Drive left
                                                                                                       // with negative
                                                                                                       // X (left)
                                .withRotationalRate(-driverController.getRightX() * TunerConstants.MaxAngularRate) // Drive
                                                                                                                   // counterclockwise
                        // with
                        // negative X (left)
                        ));

        /*
         * driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
         * driverController.b().whileTrue(drivetrain.applyRequest(
         * () -> point.withModuleDirection(
         * new Rotation2d(-driverController.getLeftY(),
         * -driverController.getLeftX()))));
         */

        //driverController.leftTrigger().onTrue(align.alignReef(AlignRequestType.LeftReefAlign));
        //driverController.rightTrigger().onTrue(align.alignReef(AlignRequestType.RightReefAlign));

        // Elevator bindings
        driverController.leftTrigger(0.05).whileTrue(elevator.getManualSupplierCommand(driverController::getLeftTriggerAxis))
                .onFalse(elevator.getManualCommand(0.0));
        driverController.rightTrigger(0.05).whileTrue(elevator.getManualSupplierCommand(() -> {
            return -1 * driverController.getRightTriggerAxis();
        })).onFalse(elevator.getManualCommand(0.0));

        driverController.y().onTrue(elevator.getManualCommand(1)).onFalse(elevator.getManualCommand(0.0)).and(isTestMode());
        driverController.a().onTrue(elevator.getManualCommand(-0.6)).onFalse(elevator.getManualCommand(0.0)).and(isTestMode());

        operatorController.y().onTrue(elevator.setTargetHeightCommand(50));        
        operatorController.b().onTrue(elevator.setTargetHeightCommand(20));        
        operatorController.a().onTrue(elevator.setTargetHeightCommand(5));        

        // Reset encoder
        operatorController.start().onTrue(elevator.getEncoderResetCommand());

        // Intake bindings
        driverController.b().onTrue(
                intake.getIntakeCommand().raceWith(outtake.feedOuttake1().andThen(outtake.feedOuttake2())).andThen(intake.getIntakeStopCommand()));

        // Outtake bindings
        driverController.x().onTrue(outtake.setFeed(0.9)).onFalse(outtake.setFeed(0));

        // Algae bindings
        driverController.leftBumper().onTrue(algae.setAlgaeGrabbers(0.5)).onFalse(algae.setAlgaeGrabbers(0));
        driverController.rightBumper().onTrue(algae.setAlgaeGrabbers(-0.5)).onFalse(algae.setAlgaeGrabbers(0));

        /*
         * // Run SysId routines when holding back/start and X/Y.
         * // Note that each routine should be run exactly once in a single log.
         * driverController.back().and(driverController.y()).whileTrue(drivetrain.
         * sysIdDynamic(Direction.kForward));
         * driverController.back().and(driverController.x()).whileTrue(drivetrain.
         * sysIdDynamic(Direction.kReverse));
         * driverController.start().and(driverController.y()).whileTrue(drivetrain.
         * sysIdQuasistatic(Direction.kForward));
         * driverController.start().and(driverController.x()).whileTrue(drivetrain.
         * sysIdQuasistatic(Direction.kReverse));
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

    public static double getDriveControllerLeftDistance() {
        return Math.sqrt(Math.pow(driverController.getLeftX(), 2) + Math.pow(driverController.getLeftY(), 2));
    }

    public static BooleanSupplier isTestMode() {
        return DriverStation::isTestEnabled;
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

    public static void setLightPattern(LightPattern pattern) {
        lightManager.setLightPattern(pattern);
    }

    public static void addError(ErrorBody error) {
        errorManager.addError(error);
    }

    public static boolean hasError(ErrorMode mode) {
        return errorManager.checkForExistingError(mode);
    }

    public static void setEnabled(boolean value) {
        isEnabled = value;
    }

    private Command runAuto = drivetrain.getAutoPath("Test");

    public Command getAutonomousCommand() {
        CommandScheduler.getInstance().schedule(elevator.getEncoderResetCommand());

        return new SequentialCommandGroup(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
                runAuto);
    }
}
