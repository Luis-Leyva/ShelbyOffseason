package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.overture.lib.sensors.CanCoderConfig;

import edu.wpi.first.units.measure.Angle;

public class IntakeConstants {
	public static final int SHOOTER_MOTOR_ID = 22;
	public static final int SHOOTER_ENCODER_ID = 21;
	public static final Angle POSITION_TOLERANCE = Degrees.of(3.0);
	public static final double GEAR_RATIO = (30.0 / 35.0);

	// Motor Config
	public static TalonFXConfiguration motorConfig() {
		return new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
								.withStatorCurrentLimit(120.0)
								.withSupplyCurrentLimitEnable(true)
								.withSupplyCurrentLimit(40.0)
								.withSupplyCurrentLowerLimit(40.0)
								.withSupplyCurrentLowerTime(0.05))
				.withVoltage(
						new VoltageConfigs()
								.withPeakForwardVoltage(12.0)
								.withPeakReverseVoltage(-12.0))
				.withMotorOutput(
						new MotorOutputConfigs()
								.withInverted(InvertedValue.CounterClockwise_Positive)
								.withNeutralMode(NeutralModeValue.Brake))
				.withSlot0(
						new Slot0Configs()
								.withKP(0.0)
								.withKD(0.0))
				.withFeedback(new FeedbackConfigs()
						.withFeedbackRemoteSensorID(SHOOTER_ENCODER_ID)
						.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
						.withSensorToMechanismRatio(1.0)
						.withRotorToSensorRatio(GEAR_RATIO));
	}

	// Encoder Config
	public static CanCoderConfig encoderConfig() {
		CanCoderConfig config = new CanCoderConfig();

		config.CanCoderId = SHOOTER_ENCODER_ID;
		config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

		return config;
	}
}
