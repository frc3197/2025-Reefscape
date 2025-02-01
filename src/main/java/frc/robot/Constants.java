package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

public class Constants {
	public static Pose2d[][] RedAlignPositions = {

			// Front Right: ZERO
			{
					// Left
					new Pose2d(13.92, 5.12, new Rotation2d(Units.degreesToRadians(-60))),

					// Right
					new Pose2d(13.59, 5.26, new Rotation2d(Units.degreesToRadians(-60)))
			},

			// Front middle: ONE
			{
					// Left
					new Pose2d(14.25, 3.75, new Rotation2d(Units.degreesToRadians(180))),

					// Right
					new Pose2d(14.25, 4.23, new Rotation2d(Units.degreesToRadians(180)))
			},

			// Back Right: FIVE
			{
					// Left
					new Pose2d(12.15, 5.12, new Rotation2d(Units.degreesToRadians(60))),

					// Right
					new Pose2d(12.49, 5.22, new Rotation2d(Units.degreesToRadians(60)))
			}
	};

	public static class ElevatorConstants {
		public static final int leftElevatorMotorId = 8;
		public static final int rightElevatorMotorId = 9;

		public static final ElevatorFeedforward lightLoadElevatorFeed = new ElevatorFeedforward(0, 0, 0);
		public static final PIDController lightLoadElevatorPID = new PIDController(0.2, 0, 0);

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
		public static final int level1Encoder = 2500;
		public static final int level2Encoder = 25000;
		public static final int level3Encoder = 44500;
		public static final int level4Encoder = 69500;
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
				.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

		public static final int algaeEncoderChannel = 3;

		// 0 degrees
		public static double algaeDownEncoder = 0.448;
		
		// 90 degrees
		public static double algaeUpEncoder = 0.185;
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
	}
}
