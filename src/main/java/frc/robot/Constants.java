package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

public class Constants {
	public static Pose2d[][] RedAlignPositions = {
			// Back Right
			{
					// Left
					new Pose2d(12.15, 5.12, new Rotation2d(Units.degreesToRadians(-60))),

					// Right
					new Pose2d(12.49, 5.22, new Rotation2d(Units.degreesToRadians(-60)))
			},

			// Front Right
			{
					// Left
					new Pose2d(13.92, 5.12, new Rotation2d(Units.degreesToRadians(60))),

					// Right
					new Pose2d(13.59, 5.26, new Rotation2d(Units.degreesToRadians(60)))
			},

			// Front middle
			{
					// Left
					new Pose2d(14.35, 3.82, new Rotation2d(0)),

					// Right
					new Pose2d(14.35, 4.23, new Rotation2d(0))
			}
	};

	public static class ElevatorConstants {
		public static final int leftElevatorId = 8;
		public static final int rightElevatorId = 9;

		public static TalonFXConfiguration leftElevatorConfig = new TalonFXConfiguration().withCurrentLimits(
				new CurrentLimitsConfigs()
						.withStatorCurrentLimit(Amps.of(20))
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimit(Amps.of(20))
						.withSupplyCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

		public static TalonFXConfiguration rightElevatorConfig = new TalonFXConfiguration().withCurrentLimits(
				new CurrentLimitsConfigs()
						.withStatorCurrentLimit(Amps.of(20))
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimit(Amps.of(20))
						.withSupplyCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
	}
}
