package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

public class Constants {

	public static class AlignPositions {
		public static class RedPositions {
			// Front right positions
			public static final Pose2d frontRightLPoseAlignRed = new Pose2d(13.846, 5.068, new Rotation2d(Units.degreesToRadians(60)));
			public static final Pose2d frontRightRPoseAlignRed = new Pose2d(13.558, 5.212, new Rotation2d(Units.degreesToRadians(60)));

			// Front center positions
			public static final Pose2d frontCenterLPoseAlignRed = new Pose2d(14.45, 3.91, new Rotation2d(Units.degreesToRadians(0)));
			public static final Pose2d frontCenterRPoseAlignRed = new Pose2d(14.45, 4.21, new Rotation2d(Units.degreesToRadians(0)));
			
			// Front left positions
			public static final Pose2d frontLeftLPoseAlignRed = new Pose2d(13.5, 2.8, new Rotation2d(Units.degreesToRadians(-60)));
			public static final Pose2d frontLeftRPoseAlignRed = new Pose2d(13.858, 3.018, new Rotation2d(Units.degreesToRadians(-60)));

			// Back left positions
			public static final Pose2d backLeftLPoseAlignRed = new Pose2d(12.551, 2.814, new Rotation2d(Units.degreesToRadians(-120)));
			public static final Pose2d backLeftRPoseAlignRed = new Pose2d(12.251, 2.970, new Rotation2d(Units.degreesToRadians(-120)));

			public static final Pose2d[][] redFeefPoses = {{frontRightLPoseAlignRed, frontRightRPoseAlignRed}, {frontCenterLPoseAlignRed, frontCenterRPoseAlignRed}, {frontLeftLPoseAlignRed, frontLeftRPoseAlignRed}, {backLeftLPoseAlignRed, backLeftRPoseAlignRed}};
		}
	}

	public static class ElevatorConstants {
		public static final int leftElevatorMotorId = 8;
		public static final int rightElevatorMotorId = 9;

		public static final ElevatorFeedforward emptyLoadElevatorFeed = new ElevatorFeedforward(0, 0.03, 0);

		public static final ElevatorFeedforward algaeLoadElevatorFeed = new ElevatorFeedforward(0, 0, 0);

		public static final PIDController emptyLoadElevatorPID = new PIDController(0.95, 0, 0);
		public static final PIDController algaeLoadElevatorPID = new PIDController(0.2, 0, 0);

		public static final TalonFXConfiguration leftElevatorMotorConfig = new TalonFXConfiguration().withCurrentLimits(
				new CurrentLimitsConfigs()
						.withStatorCurrentLimit(Amps.of(40))
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimit(Amps.of(40))
						.withSupplyCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

		public static final TalonFXConfiguration rightElevatorMotorConfig = new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimit(Amps.of(40))
								.withStatorCurrentLimitEnable(true)
								.withSupplyCurrentLimit(Amps.of(40))
								.withSupplyCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

		public static final int elevatorEncoderChannelA = 1;
		public static final int elevatorEncoderChannelB = 2;

		// Measured in rotations
		public static final int encoderLowValue = 0;
		public static final int encoderHighValue = 100;

		// Measured in cm
		public static final int elevatorLowHeight = 30;
		public static final int elevatorHighHeight = 200;

		public static final int loadingStationEncoder = 100;
		public static final int level1Encoder = 12500;
		public static final int level2Encoder = 12500;
		public static final int level3Encoder = 24500;
		public static final int level4Encoder = 40700;
		public static final int alignIdleEncoder = 6500;

		public static final int lowAlgaeEncoder = 21500;
		public static final int highAlgaeEncoder = 32000;
	}

	public static class AlgaeConstants {
		public static final int deployMotorId = 10;
		public static final int spinMotorId = 11;

		public static final int leftGrabberMotorId = 15;
		public static final int rightGrabberMotorId = 16;

		public static final int algaeLaserCanId = 1;
		public static final double algaeSensorDistance = 15;

		public static final TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration().withCurrentLimits(
				new CurrentLimitsConfigs()
						.withStatorCurrentLimit(Amps.of(110))
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimit(Amps.of(110))
						.withSupplyCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
				.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(30));
		// Check gear ratio for deploy motor

		public static final int algaeEncoderChannel = 3;

		// 0 degrees
		public static double algaeDownEncoder = 0.628;

		// 90 degrees
		public static double algaeUpEncoder = 0.37;

		public static ArmFeedforward emptyLoadArmFeedForward = new ArmFeedforward(0.0, 0.01, 0);
		public static ArmFeedforward algaeLoadArmFeedForward = new ArmFeedforward(0, 0, 0);

		public static PIDController emptyLoadArmPID = new PIDController(0.05, 0, 0);
		public static PIDController algaeLoadArmPID = new PIDController(0, 0, 0);
	}

	public static class IntakeConstants {
		public static final int leftIntakeMotorId = 12;
		public static final int rightIntakeMotorId = 13;

		public static final double leftMotorSpeed = 0.7;
		public static final double rightMotorSpeed = 0.8;

		public static final double intakeSensorDelay = 0.1;

		public static final TalonFXConfiguration leftIntakeMotorConfig = new TalonFXConfiguration().withCurrentLimits(
				new CurrentLimitsConfigs()
						.withStatorCurrentLimit(Amps.of(20))
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimit(Amps.of(20))
						.withSupplyCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

		public static final TalonFXConfiguration rightIntakeMotorConfig = new TalonFXConfiguration().withCurrentLimits(
				new CurrentLimitsConfigs()
						.withStatorCurrentLimit(Amps.of(20))
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimit(Amps.of(20))
						.withSupplyCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

	}

	public static class OuttakeConstants {
		public static final int outtakeMotorId = 14;
		public static final int outtakeLaserCan = 0;

		public static final double sensorRange = 100;

		public static final double outtakeFeedSpeed = 0.25;
		public static final double outtakeSpitSpeed = 1;

		public static final TalonFXConfiguration outtakeMotorConfig = new TalonFXConfiguration().withCurrentLimits(
				new CurrentLimitsConfigs()
						.withStatorCurrentLimit(Amps.of(50))
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimit(Amps.of(50))
						.withSupplyCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

	}

	public static class VisionConstants {
		public static String limelightFrontName = "";

		public static PIDController strafeAlignPID = new PIDController(0.135, 0, 0);
	}
}
