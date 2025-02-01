// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.lang.annotation.Documented;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.enums.*;
import frc.robot.managersubsystems.LightManager;
import frc.robot.managersubsystems.ErrorManager;
import frc.robot.managersubsystems.AlertManager;
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
    private final static AlertManager alertManager = new AlertManager();

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Vision poseEstimator = new Vision(drivetrain);
    private final Align align = new Align(drivetrain);
    private final Elevator elevator = new Elevator();
    private final Outtake outtake = new Outtake();
    private final Algae algae = new Algae();

    // Controllers
    private final static CommandXboxController driverController = new CommandXboxController(0);
    private final static CommandXboxController operatorController = new CommandXboxController(1);
    private final static CommandXboxController[] controllers = { driverController, operatorController };

    // Robot statuses
    private static boolean isEnabled = false;
    private static boolean isTestMode = false;
    private static boolean hasAlgae = false;
    private static RobotMode robotMode;

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
                                .withRotationalRate(driverController.getRightX() * TunerConstants.MaxAngularRate) // Drive
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

        // -------------------------------------------------------------------------
        // Drive & align bindings
        // -------------------------------------------------------------------------

        driverController.leftTrigger().onTrue(
                align.alignReefRough(AlignRequestType.LEFT_REEF_ALIGN).onlyWhile(() -> {
                    return this.getDriveControllerLeftDistance() < 0.35;
                }));
        driverController.rightTrigger().onTrue(
                align.alignReefRough(AlignRequestType.RIGHT_REEF_ALIGN).onlyWhile(() -> {
                    return this.getDriveControllerLeftDistance() < 0.35;
                }));
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // -------------------------------------------------------------------------
        // Elevator bindings
        // -------------------------------------------------------------------------

        /*
         * driverController.leftTrigger(0.05)
         * .whileTrue(elevator.getManualSupplierCommand(driverController::
         * getLeftTriggerAxis))
         * .onFalse(elevator.getManualCommand(0.0));
         * driverController.rightTrigger(0.05).whileTrue(elevator.
         * getManualSupplierCommand(() -> {
         * return -1 * driverController.getRightTriggerAxis();
         * })).onFalse(elevator.getManualCommand(0.0));
         */

        /*
         * driverController.y().onTrue(elevator.getManualCommand(1)).onFalse(elevator.
         * getManualCommand(0.0))
         * .and(isTestMode());
         * driverController.a().onTrue(elevator.getManualCommand(-0.6)).onFalse(elevator
         * .getManualCommand(0.0))
         * .and(isTestMode());
         */

        operatorController.x().onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level4Encoder));
        operatorController.y().onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level3Encoder));
        operatorController.b().onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level2Encoder));
        operatorController.a()
                .onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder));

        operatorController.povUp().onTrue(elevator.setTargetHeightCommand(58000));
        operatorController.povRight().onTrue(elevator.setTargetHeightCommand(38600));

        // Reset encoder
        operatorController.start().onTrue(elevator.getEncoderResetCommand());

        // -------------------------------------------------------------------------
        // Intake bindings
        // -------------------------------------------------------------------------
        driverController.b().onTrue(
                getIntakeCommand());

        // -------------------------------------------------------------------------
        // Outtake bindings
        // -------------------------------------------------------------------------
        driverController.x().onTrue(outtake.setFeed(0.9)).onFalse(outtake.setFeed(0));

        // -------------------------------------------------------------------------
        // Algae bindings
        // -------------------------------------------------------------------------
        driverController.leftBumper().onTrue(algae.setAlgaeGrabberSpeedCommand(0.85))
                .onFalse(algae.setAlgaeGrabberSpeedCommand(0));
        driverController.rightBumper().onTrue(algae.setAlgaeGrabberSpeedCommand(-1.0))
                .onFalse(algae.setAlgaeGrabberSpeedCommand(0));

        driverController.povUp().onTrue(algae.setDeploySpeedCommand(0.3)).onFalse(algae.setDeploySpeedCommand(0.0));
        driverController.povDown().onTrue(algae.setDeploySpeedCommand(-0.3)).onFalse(algae.setDeploySpeedCommand(0.0));

        driverController.y().onTrue(algae.setTargetAngleDegrees(80));
        driverController.povRight().onTrue(algae.setTargetAngleDegrees(10));
        driverController.a().onTrue(algae.setTargetAngleDegrees(-15));

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

    private double getDriveControllerLeftDistance() {
        return Math.sqrt(Math.pow(driverController.getLeftX(), 2) + Math.pow(driverController.getLeftY(), 2));
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

    public static void addError(ErrorBody error) {
        errorManager.addError(error);
    }

    public static boolean hasErrors() {
        return errorManager.hasErrors();
    }

    public static boolean hasError(ErrorMode mode) {
        return errorManager.checkForExistingError(mode);
    }

    public static void addAlert(AlertBody alert) {
        alertManager.addAlert(alert);
    }

    public static boolean hasAlerts() {
        return alertManager.hasAlerts();
    }

    public static boolean hasAlert(AlertMode mode) {
        return alertManager.checkForExistingAlert(mode);
    }

    public static void setEnabled(boolean value) {
        isEnabled = value;
    }

    public static boolean getEnabled() {
        return isEnabled;
    }

    public static void setTestMode(boolean value) {
        isEnabled = value;
    }

    public static boolean getTestMode() {
        return isEnabled;
    }

    public static void setHasAlgae(boolean value) {
        hasAlgae = value;
    }

    public static boolean getHasAlgae() {
        return hasAlgae;
    }

    public static void setRobotMode(RobotMode value) {
        robotMode = value;
    }

    public static RobotMode getRobotMode() {
        return robotMode;
    }

    public Command getIntakeCommand() {
        return new SequentialCommandGroup(
                outtake.feedOuttake(0.5),
                Commands.waitUntil(outtake.isBridgingSupplier()),
                outtake.feedOuttake(0.2),
                Commands.waitUntil(() -> {
                    return !outtake.isBridgingSupplier().getAsBoolean();
                }),
                new InstantCommand(() -> {
                    if (!RobotContainer.hasAlert(AlertMode.ACQUIRED_CORAL)) {
                        RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_CORAL, 1.5));
                    }
                }),
                outtake.feedOuttake(-0.1),
                new WaitCommand(0.1),
                outtake.feedOuttake(0.0));
    }

    private Command autoCommand = drivetrain.getAutoPath("Straight");

    public Command getAutonomousCommand() {
        CommandScheduler.getInstance().schedule(elevator.getEncoderResetCommand());

        return new SequentialCommandGroup(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
                drivetrain.runOnce(() -> drivetrain.resetNewPose(new Pose2d(2, 6, new Rotation2d(0)))),
                autoCommand);
    }
}
