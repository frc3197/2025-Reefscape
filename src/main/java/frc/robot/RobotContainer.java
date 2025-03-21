// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.io.IOException;
import java.lang.annotation.Documented;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.AlignBarge;
import frc.robot.commands.AlignReef;
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
            .withDeadband(TunerConstants.MaxSpeed * 0.025).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.025) // Add
                                                                                                                       // a
                                                                                                                       // 10%
                                                                                                                       // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);

    // Managers
    private final static LightManager lightManager = new LightManager();
    private final static ErrorManager errorManager = new ErrorManager();
    private final static AlertManager alertManager = new AlertManager();

    // Subsystems
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final static Vision vision = new Vision(drivetrain);
    private final static Align align = new Align(drivetrain);
    private final static Elevator elevator = new Elevator();
    private final static Outtake outtake = new Outtake();
    private final static Algae algae = new Algae();
    private final static Climber climber = new Climber();
    private final AutoLookup autoLookup = new AutoLookup(drivetrain, vision, align, elevator, outtake, algae);

    // Controllers
    private final static CommandXboxController driverController = new CommandXboxController(0);
    private final static CommandXboxController operatorController = new CommandXboxController(1);
    private final static CommandXboxController climbController = new CommandXboxController(2);
    private final static CommandXboxController[] controllers = { driverController, operatorController };

    // Robot statuses
    private static boolean isEnabled = false;
    private static boolean isTestMode = false;
    private static boolean hasAlgae = false;
    private static RobotMode robotMode;
    private static NetworkTable autoTable = NetworkTableInstance.getDefault().getTable("Auto"); 

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

        // -------------------------------------------------------------------------
        // Drive & align bindings
        // -------------------------------------------------------------------------

        /*
         * // X wheels
         * driverController.rightBumper().whileTrue(drivetrain.applyRequest(
         * () -> point.withModuleDirection(
         * new Rotation2d(-driverController.getLeftY(),
         * -driverController.getLeftX()))));
         */

        // Bump up elevator
        operatorController.back().onTrue(elevator.setTargetHeightCommand(() -> {
            return elevator.getTargetHeight() + 2.5;
        }));

        operatorController.rightStick().onTrue(elevator.setTargetHeightCommand(() -> {
            return elevator.getTargetHeight() - 1.5;
        }));

        driverController.rightStick().onTrue(outtake.setFeed(-0.2)).onFalse(outtake.setFeed(0.0));
        driverController.leftStick().onTrue(outtake.setFeed(0.28)).onFalse(outtake.setFeed(0.0));

        driverController.leftBumper()
                .whileTrue(Commands.run(() -> drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0.45, 0))))
                .onFalse(new InstantCommand(() -> drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0))));
        driverController.rightBumper()
                .whileTrue(Commands.run(() -> drivetrain.driveRobotRelative(new ChassisSpeeds(0, -0.45, 0))))
                .onFalse(new InstantCommand(() -> drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0))));

        driverController.leftTrigger().whileTrue(new AlignReef(align, AlignRequestType.LEFT_REEF_ALIGN,
                new ChassisSpeeds(0.95, 1.0, 1.85), new Translation3d(0.015, 0.015, 0.01)));

        driverController.rightTrigger().whileTrue(new AlignReef(align, AlignRequestType.RIGHT_REEF_ALIGN,
                new ChassisSpeeds(0.85, 0.85, 1.5), new Translation3d(0.015, 0.015, 0.01)));

        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // -------------------------------------------------------------------------
        // Elevator bindings
        // -------------------------------------------------------------------------

        operatorController.x().onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level4Encoder + 0));
        operatorController.y()
                .onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level3Encoder + 0));
        operatorController.b()
                .onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level2Encoder + 0));
        operatorController.a()
                .onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder));

        // Zero encoder
        operatorController.start().onTrue(elevator.getEncoderResetCommand());

        // -------------------------------------------------------------------------
        // Intake bindings
        // -------------------------------------------------------------------------
        driverController.b().onTrue(
                getIntakeCommand());

        // -------------------------------------------------------------------------
        // Outtake bindings
        // -------------------------------------------------------------------------
        driverController.x().onTrue(outtake.setFeed(0.44)).onFalse(outtake.setFeed(0));

        // -------------------------------------------------------------------------
        // Algae bindings
        // -------------------------------------------------------------------------
        operatorController.leftBumper().onTrue(algae.setAlgaeGrabberSpeedCommand(1.0))
                .onFalse(algae.setAlgaeGrabberSpeedCommand(0));
        operatorController.rightBumper().onTrue(algae.setAlgaeGrabberSpeedCommand(-1.0))
                .onFalse(algae.setAlgaeGrabberSpeedCommand(0));

        driverController.povUp().onTrue(algae.setDeploySpeedCommand(0.3)).onFalse(algae.setDeploySpeedCommand(0.0));
        driverController.povDown().onTrue(algae.setDeploySpeedCommand(-0.3)).onFalse(algae.setDeploySpeedCommand(0.0));

        operatorController.povUp()
                .onTrue(algae.setTargetAngleDegrees(90).andThen(algae.setAlgaeGrabberSpeedCommand(0.0)));
        operatorController.povRight().onTrue(algae.setTargetAngleDegrees(10));
        operatorController.povDown().onTrue(algae.setTargetAngleDegrees(-5));

        driverController.a().onTrue(getAlgaeLowCommand());
        operatorController.povLeft().onTrue(getAlgaeIntakeStagedCommand());
        driverController.y().onTrue(getAlgaeHighCommand());

        operatorController.rightTrigger().onTrue(getAlgaeIntakeFloorCommand());
        operatorController.leftTrigger().onTrue(getAlgaeScoreBargeInitial()).onFalse(getAlgaeScoreBargeFinal());

        // -------------------------------------------------------------------------
        // DRIVE LOGGING & TUNING
        // -------------------------------------------------------------------------
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // extraController.back().and(extraController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // extraController.back().and(extraController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // extraController.start().and(extraController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // extraController.start().and(extraController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);

        // -------------------------------------------------------------------------
        // EXTRA TESTING COMMANDS
        // -------------------------------------------------------------------------

        climbController.leftTrigger(0.2).whileTrue(climber.setClimberArmSpeed(
            climbController::getLeftTriggerAxis))
                .onFalse(climber.setClimberArmSpeed(0));
        climbController.rightTrigger(0.2).whileTrue(climber.setClimberSpeedValue(
            -1*climbController.getRightTriggerAxis()
        )).onFalse(climber.setClimberArmSpeed(0));

        climbController.rightBumper().onTrue(climber.setClimberSpeedValue(-0.35)).onFalse(climber.setClimberSpeedValue(0));

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
        isTestMode = value;
    }

    public static boolean getTestMode() {
        return isTestMode;
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

    public static double[] getBestAlignCameraTarget() {
        return vision.getBestAlignCameraTarget();
    }

    public static Command getIntakeCommand() {
        return new SequentialCommandGroup(
                // elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder),
                outtake.feedOuttake(0.45),
                Commands.waitUntil(outtake.detectsCoralSupplier()),
                outtake.feedOuttake(0.08),
                Commands.waitUntil(() -> {
                    return !outtake.detectsCoralSupplier().getAsBoolean();
                }),
                new InstantCommand(() -> {
                    if (!RobotContainer.hasAlert(AlertMode.ACQUIRED_CORAL)) {
                        RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_CORAL, 0.65));
                    }
                }),
                outtake.feedOuttake(-0.23),
                new WaitCommand(0.0967),
                outtake.feedOuttake(0.0));
    }

    public static Command getAutoIntakeCommand() {
        return new SequentialCommandGroup(
                // elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder),
                outtake.feedOuttake(0.45),
                Commands.waitUntil(outtake.detectsCoralSupplier()),
                outtake.feedOuttake(0.08),
                Commands.waitUntil(() -> {
                    return !outtake.detectsCoralSupplier().getAsBoolean();
                }).withTimeout(1.2),
                new InstantCommand(() -> {
                    if (!RobotContainer.hasAlert(AlertMode.ACQUIRED_CORAL)) {
                        RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_CORAL, 0.25));
                    }
                }));
    }

    public Command getAutonomousCommand() {

        return new SequentialCommandGroup(
                elevator.getEncoderResetCommand(),
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),

                // Set pose, better and more reliable than vision for now in case of miss
                drivetrain.runOnce(() -> {
                    if (isRed())
                        drivetrain.resetPose(new Pose2d(10.321, 1.915, new Rotation2d(Units.degreesToRadians(180))));
                    else
                        /*
                         * drivetrain.resetNewPose(new Pose2d(7.300, 4.180,
                         * new Rotation2d(Units.degreesToRadians(0))));
                         */
                        drivetrain.resetPose(new Pose2d(7.229, 5.416, new Rotation2d(Units.degreesToRadians(0))));
                }),
                //autoLookup.getAuto(autoTable.getEntry("autoMode").getString("Nothing")));
                autoLookup.getAuto("Left 3 Piece"));

        // return new PathPlannerAuto("FarAuto");
    }

    public static Command getScoreSequenceL4Command() {
        return elevator
                .setTargetHeightCommand(Constants.ElevatorConstants.level4Encoder + 0).andThen(() -> {
                    drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
                })
                .andThen(Commands.waitUntil(() -> {
                    return elevator.getPositionError() < 2.9;
                }).withTimeout(2)).andThen(new WaitCommand(0.25).andThen(outtake.feedOuttake(0.25)))
                .andThen(new WaitCommand(0.22)).andThen(elevator
                        .setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder))
                .andThen(outtake.feedOuttake(0))
                .andThen(() -> {
                    drivetrain.driveRobotRelative(new ChassisSpeeds(-0.4, 0, 0));
                })
                .andThen(new WaitCommand(0.0))
                .andThen(() -> {
                    // drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
                });
    }

    public static Command getScoreSequenceL3Command() {
        return elevator
                .setTargetHeightCommand(Constants.ElevatorConstants.level3Encoder).andThen(Commands.waitUntil(() -> {
                    return elevator.getPositionError() < 50;
                }).withTimeout(1.75)).andThen(outtake.feedOuttake(0.8)).andThen(new WaitCommand(0.5)).andThen(elevator
                        .setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder))
                .andThen(new WaitCommand(0.75));
    }

    public static Command getAlgaeScoreBargeInitial() {
        return algae.setTargetAngleDegrees(68.5)
                .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level4Encoder + 0));
    }

    public static Command getAlgaeScoreBargeFinal() {
        return (algae.setAlgaeGrabberSpeedCommand(-0.75)).andThen(Commands.waitSeconds(0.07))
                .andThen(algae.setAlgaeGrabberSpeedCommand(1, 1)).andThen(Commands.waitSeconds(0.45))
                .andThen(algae.setTargetAngleDegrees(85)).andThen(algae.setAlgaeGrabberSpeedCommand(0))
                .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder));
    }

    public static Command getAlgaeIntakeFloorCommand() {
        return new SequentialCommandGroup(
                algae.setTargetAngleDegrees(-15.5)
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder))
                        .andThen(algae.setAlgaeGrabberSpeedCommand(-0.8))
                        .andThen(Commands.waitUntil(algae.getHasAlgaeSupplier()))
                        .andThen(() -> {
                            RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_ALGAE, 0.75));
                        })
                        .andThen(algae.setAlgaeGrabberSpeedCommand(0))
                        .andThen(algae.setTargetAngleDegrees(40))
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder)));
    }

    public static Command getAlgaeIntakeStagedCommand() {
        return new SequentialCommandGroup(
                algae.setTargetAngleDegrees(18)
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder))
                        .andThen(algae.setAlgaeGrabberSpeedCommand(-0.8))
                        .andThen(Commands.waitUntil(algae.getHasAlgaeSupplier()))
                        .andThen(() -> {
                            RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_ALGAE, 0.75));
                        })
                        .andThen(algae.setAlgaeGrabberSpeedCommand(0))
                        .andThen(algae.setTargetAngleDegrees(40))
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder)));
    }

    public static Command getAlgaeLowCommand() {
        return new SequentialCommandGroup(
                algae.setTargetAngleDegrees(-8)
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.lowAlgaeEncoder))
                        .andThen(algae.setAlgaeGrabberSpeedCommand(-0.8))
                        .andThen(Commands.waitUntil(algae.getHasAlgaeSupplier()))
                        .andThen(() -> {
                            RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_ALGAE, 0.75));
                        })
                        .andThen(algae.setAlgaeGrabberSpeedCommand(0))
                        .andThen(new WaitCommand(0.5))
                        .andThen(algae.setTargetAngleDegrees(85))
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.lowAlgaeEncoder - 8))
                        .andThen(new WaitCommand(0.85))
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder)));
    }

    public static Command getAlgaeHighCommand() {
        return new SequentialCommandGroup(
                algae.setTargetAngleDegrees(-3)
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.highAlgaeEncoder))
                        .andThen(algae.setAlgaeGrabberSpeedCommand(-0.8))
                        .andThen(Commands.waitUntil(algae.getHasAlgaeSupplier()))
                        .andThen(() -> {
                            RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_ALGAE, 0.75));
                        })
                        .andThen(algae.setAlgaeGrabberSpeedCommand(0))
                        .andThen(new WaitCommand(0.5))
                        .andThen(algae.setTargetAngleDegrees(85))
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.highAlgaeEncoder - 8))
                        .andThen(new WaitCommand(0.85))
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder)));
    }
}
