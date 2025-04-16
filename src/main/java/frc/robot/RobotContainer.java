// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.io.IOException;
import java.lang.annotation.Documented;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    private final SwerveRequest.RobotCentric algaeDrive = new SwerveRequest.RobotCentric()
            .withDeadband(TunerConstants.MaxSpeed * 0.025).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.025) // Add
                                                                                                                         // a
                                                                                                                         // 10%
                                                                                                                         // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
    // private static NetworkTable autoTable =
    // NetworkTableInstance.getDefault().getTable("Auto");

    private SendableChooser<String> autoSelecter = new SendableChooser<String>();

    private static double maxSpeedX = TunerConstants.MaxSpeed;

    private Trigger netAligned;

    public RobotContainer() {
        autoSelecter.addOption("Left 3", "Left 3");
        autoSelecter.addOption("Center-L High-Net-Net", "Center-L High-Net-Net");

        SmartDashboard.putData("AUTO MODE", autoSelecter);

        configureBindings();
    }

    private void configureBindings() {

        netAligned = new Trigger(() -> {
            return alignedNet();
        }).onTrue(Commands.runOnce(() -> {
            setRobotMode(RobotMode.NET_ALIGNED);
        })).onFalse(Commands.runOnce(() -> {
            if (getRobotMode() == RobotMode.NET_ALIGNED) {
                setRobotMode(RobotMode.NONE);
            }
        }));

        drivetrain.setDefaultCommand(
                // maxSpeedX
                getDrivingCommand());

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
            return elevator.getTargetHeight() + 1.5;
        }));

        operatorController.rightStick().onTrue(elevator.setTargetHeightCommand(() -> {
            return elevator.getTargetHeight() - 1.5;
        }));

        /*
         * driverController.rightStick().onTrue(Commands.runOnce(() ->
         * {drivetrain.setDefaultCommand(
         * // maxSpeedX
         * getAlgaeDriveCommand());}, drivetrain)).onFalse(Commands.runOnce(() ->
         * {drivetrain.setDefaultCommand(
         * // maxSpeedX
         * getDrivingCommand());}, drivetrain));
         */
        driverController.leftStick().onTrue(outtake.setFeed(-0.2).andThen(outtake.setStar(0.45))).onFalse(outtake.setFeed(0.0).andThen(outtake.setStar(0.0)));
        // driverController.leftStick().onTrue(outtake.setFeed(0.28)).onFalse(outtake.setFeed(0.0));

        driverController.leftBumper()
                .whileTrue(Commands.run(() -> drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0.45, 0))))
                .onFalse(new InstantCommand(() -> drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0))));
        driverController.rightBumper()
                .whileTrue(Commands.run(() -> drivetrain.driveRobotRelative(new ChassisSpeeds(0, -0.45, 0))))
                .onFalse(new InstantCommand(() -> drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0))));

        driverController.leftTrigger().whileTrue(new AlignReef(align, AlignRequestType.LEFT_REEF_ALIGN,
                new ChassisSpeeds(0.95+0.5, 1.0+0.5, 3.5), new Translation3d(0.015, 0.015, 0.01)));

        driverController.rightTrigger().whileTrue(new AlignReef(align, AlignRequestType.RIGHT_REEF_ALIGN,
                new ChassisSpeeds(0.85+0.5, 0.85+0.5, 3.5), new Translation3d(0.015, 0.015, 0.01)));

        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // -------------------------------------------------------------------------
        // Elevator bindings
        // -------------------------------------------------------------------------

        operatorController.x()
                .onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level4Encoder + 0.5));
        operatorController.y()
                .onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level3Encoder + 0));
        operatorController.b()
                .onTrue(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level2Encoder + 0));
        operatorController.a()
                .onTrue(elevator.setTargetHeightCommand(0.15));

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
        driverController.x().onTrue(outtake.setFeed(0.44).andThen(outtake.setStar(-0.45))).onFalse(outtake.setFeed(0).andThen(outtake.setStar(0.0)));

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
        operatorController.leftTrigger().onTrue(getAlgaeScoreNetInitial())
                .onFalse(Commands.runOnce(() -> {
                    addRumble(0, 0.5, 0.5, RumbleType.kBothRumble);
                }).andThen(getAlgaeScoreNetFinal()).andThen(new WaitCommand(0.3)));
        // .andThen(getClampSpeedXCommand(TunerConstants.MaxSpeed))

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
                .onFalse(climber.setClimberSpeedValue(0));
        /*
         * climbController.rightTrigger(0.2).whileTrue(climber.setClimberSpeedValue(
         * -1*climbController.getRightTriggerAxis()
         * )).onFalse(climber.setClimberArmSpeed(0));
         */

        climbController.rightTrigger().onTrue(climber.setClimberSpeedValue(-1))
                .onFalse(climber.setClimberSpeedValue(0));

        climbController.back().and(climbController.a()).onTrue(autoLookup.getTestAuto());

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

    /*
     * public static double[] getBestAlignCameraTarget() {
     * return vision.getBestAlignCameraTarget();
     * }
     */

    public static Command getIntakeCommand() {
        return new SequentialCommandGroup(
                // elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder),
                elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder),
                outtake.setStar(-0.5),
                outtake.feedOuttake(0.425),
                Commands.waitUntil(outtake.detectsCoralSupplier()),
                outtake.feedOuttake(0.10),
                Commands.waitUntil(() -> {
                    return !outtake.detectsCoralSupplier().getAsBoolean();
                }),
                new InstantCommand(() -> {
                    if (!RobotContainer.hasAlert(AlertMode.ACQUIRED_CORAL)) {
                        RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_CORAL, 0.65));
                    }
                }),
                elevator.setTargetHeightCommand(0.2),
                outtake.setStar(0.0),
                outtake.feedOuttake(-0.3),
                new WaitCommand(0.0987),
                outtake.feedOuttake(0.0)
                );
    }

    public static Command getAutoIntakeCommand() {
        return new SequentialCommandGroup(
                // elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder),
                outtake.setStar(-0.65),
                outtake.feedOuttake(0.775),
                Commands.waitUntil(outtake.detectsCoralSupplier()),
                outtake.feedOuttake(0.10),
                Commands.waitUntil(() -> {
                    return !outtake.detectsCoralSupplier().getAsBoolean();
                }).withTimeout(1.2),
                outtake.setStar(0.0),
                new InstantCommand(() -> {
                    if (!RobotContainer.hasAlert(AlertMode.ACQUIRED_CORAL)) {
                        RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_CORAL, 0.25));
                    }
                }));
    }

    public Command getAutonomousCommand() {

        return new WaitCommand(0.01).andThen(new SequentialCommandGroup(
                elevator.getEncoderResetCommand(),
                // drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),

                // Set pose, better and more reliable than vision for now in case of miss
                // autoLookup.getAuto(autoTable.getEntry("autoMode").getString("Nothing")));
                autoLookup.getAuto(autoSelecter.getSelected())

        ));

        // return new PathPlannerAuto("FarAuto");
    }

    public static Command getScoreSequenceL4Command() {
        return elevator
                .setTargetHeightCommand(Constants.ElevatorConstants.level4Encoder + 0.8).andThen(() -> {
                    drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
                })
                .andThen(Commands.waitUntil(() -> {
                    return elevator.getPositionError() < 3.8;
                }).withTimeout(2)).andThen(new WaitCommand(0.0).andThen(outtake.feedOuttake(0.375)))
                .andThen(new WaitCommand(0.125)).andThen(elevator
                        .setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder))
                .andThen(outtake.feedOuttake(0))
                .andThen(new WaitCommand(0.0))
                .andThen(() -> {
                    // drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
                });
    }

    public static Command getScoreSequenceL4CommandFIRST() {
        return elevator
                .setTargetHeightCommand(Constants.ElevatorConstants.level4Encoder).andThen(() -> {
                })
                .andThen(Commands.waitUntil(() -> {
                    return elevator.getPositionError() < 2.7;
                }).withTimeout(2)).andThen(new WaitCommand(0.15).andThen(outtake.feedOuttake(0.325)))
                .andThen(new WaitCommand(0.25)).andThen(elevator
                        .setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder))
                .andThen(outtake.feedOuttake(0))
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

    public static Command getAlgaeScoreNetInitial() {
        return algae.setTargetAngleDegrees(71.25)
                .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.level4Encoder + 2));
        // .andThen(getClampSpeedXCommand(1.75))
    }

    public static Command getAlgaeScoreNetFinal() {
        return (algae.setAlgaeGrabberSpeedCommand(-0.75)).andThen(Commands.waitSeconds(0.07))
                .andThen(algae.setAlgaeGrabberSpeedCommand(1, 1)).andThen(Commands.waitSeconds(0.45))
                .andThen(algae.setTargetAngleDegrees(85)).andThen(algae.setAlgaeGrabberSpeedCommand(0))
                .andThen(elevator.setTargetHeightCommand(0.2));
    }

    public static Command getAlgaeIntakeFloorCommand() {
        return new SequentialCommandGroup(
                algae.setTargetAngleDegrees(-20.5)
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.loadingStationEncoder))
                        .andThen(algae.setAlgaeGrabberSpeedCommand(-0.8))
                        .andThen(Commands.waitUntil(algae.getHasAlgaeSupplier()))
                        .andThen(() -> {
                            RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_ALGAE, 0.75));
                        })
                        .andThen(algae.setAlgaeGrabberSpeedCommand(0))
                        .andThen(algae.setTargetAngleDegrees(65))
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
                        .andThen(algae.setTargetAngleDegrees(70))
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
                        .andThen(algae.setAlgaeGrabberSpeedCommand(-0.2))
                        .andThen(new WaitCommand(0.5))
                        .andThen(algae.setAlgaeGrabberSpeedCommand(0))
                        .andThen(algae.setTargetAngleDegrees(80))
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.lowAlgaeEncoder - 15))
                        .andThen(new WaitCommand(0.7))
                        .andThen(elevator.setTargetHeightCommand(0.15)));
    }

    public static Command getAlgaeLowCommandAUTOFIRST() {
        return new SequentialCommandGroup(
                algae.setTargetAngleDegrees(-8)
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.lowAlgaeEncoder))
                        .andThen(algae.setAlgaeGrabberSpeedCommand(-0.8))
                        .andThen(Commands.waitUntil(algae.getHasAlgaeSupplier()))
                        .andThen(() -> {
                            RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_ALGAE, 0.75));
                        }));
    }

    public static Command getAlgaeCommandAUTOSECOND() {
        return new SequentialCommandGroup(
                algae.setAlgaeGrabberSpeedCommand(0)
                        .andThen(new WaitCommand(0.5))
                        .andThen(algae.setTargetAngleDegrees(85))
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.lowAlgaeEncoder - 15))
                        .andThen(new WaitCommand(0.7))
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
                        .andThen(algae.setAlgaeGrabberSpeedCommand(-0.2))
                        .andThen(new WaitCommand(0.5))
                        .andThen(algae.setAlgaeGrabberSpeedCommand(0))
                        .andThen(algae.setTargetAngleDegrees(75))
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.highAlgaeEncoder - 15))
                        .andThen(new WaitCommand(0.7))
                        .andThen(elevator.setTargetHeightCommand(0.15)));
    }

    public static Command getAlgaeHighCommandALGAEFIRST() {
        return new SequentialCommandGroup(
                algae.setTargetAngleDegrees(-3)
                        .andThen(elevator.setTargetHeightCommand(Constants.ElevatorConstants.highAlgaeEncoder))
                        .andThen(algae.setAlgaeGrabberSpeedCommand(-0.8))
                        .andThen(Commands.waitUntil(algae.getHasAlgaeSupplier()))
                        .andThen(() -> {
                            RobotContainer.addAlert(new AlertBody(AlertMode.ACQUIRED_ALGAE, 0.75));
                        }));
    }

    /*
     * private static Command getClampSpeedXCommand(double value) {
     * return Commands.runOnce(() -> {
     * maxSpeedX = value;
     * });
     * }
     */

    public static boolean alignedNet() {
        Pose2d pose = drivetrain.getState().Pose;

        double xSpeed = drivetrain.getState().Speeds.vxMetersPerSecond;
        double ySpeed = drivetrain.getState().Speeds.vyMetersPerSecond;

        if (Math.abs(ySpeed) > 4.0) {
            // return false;
        }

        return (pose.getX() + (xSpeed * 0.375)) > 7.15 && (pose.getX() + (xSpeed * -0.375)) < 10.45;
    }

    public static boolean alignAlgae() {
        return driverController.rightStick().getAsBoolean();
    }

    public static double getAlgaeYaw() {
        if(!vision.seesAlgae()) {
            setRobotMode(RobotMode.SEES_NO_ALGAE);
        }

        try {
            if(vision.seesAlgae()) {
            return vision.getAlgaeYaw()/-9.0;
            } else {
                return driverController.getRightX();
            }
        } catch (Exception e) {
            return driverController.getRightX();
        }
    }

    private Command getDrivingCommand() {

        return drivetrain
                .applyRequest(driveSupplier()
                // Drive
                // counterclockwise
                // with
                // negative X (left)
                );
    }

    public Supplier<SwerveRequest> driveSupplier() {
        return () -> {
            if (alignAlgae()) {
                return algaeDrive
                        .withVelocityX(Math.sqrt(
                                Math.pow(driverController.getLeftX(), 2) + Math.pow(driverController.getLeftY(), 2)) * TunerConstants.MaxSpeed) // Drive
                        // forward
                        // with
                        // negative Y
                        // (forward)
                        .withVelocityY(0) // Drive
                                          // left
                                          // with
                                          // negative
                                          // X
                                          // (left)
                        .withRotationalRate(
                            getAlgaeYaw());
            } else {
                if(getRobotMode() == RobotMode.SEES_ALGAE || getRobotMode() == RobotMode.SEES_NO_ALGAE) {
                    setRobotMode(RobotMode.NONE);
                }
                return drive
                        .withVelocityX(-driverController.getLeftY() * TunerConstants.MaxSpeed) // Drive
                        // forward
                        // with
                        // negative Y
                        // (forward)
                        .withVelocityY(-driverController.getLeftX() * TunerConstants.MaxSpeed) // Drive
                                                                                               // left
                                                                                               // with
                                                                                               // negative
                                                                                               // X
                                                                                               // (left)
                        .withRotationalRate(
                                -driverController.getRightX() * TunerConstants.MaxAngularRate);
            }
        };
    }
}
