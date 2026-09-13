package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
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

	// Gear Ratio
	public static final double kGearRatio = (0.0 / 0.0);

	// Motion Magic
	public static final double kCruiseVelocity = 0.0;
	public static final double kAcceleration = 0.0;

	// Device IDs
	public static final int kMotorID = 0;
	public static final int kEncoderID = 0;

	// Tolerances
	public static final Angle kAngleTolerance = Degrees.of(0.0);

	// Positions
	public static final Angle kClosedPosition = Degrees.of(0.0);
	public static final Angle kOpenPosition = Degrees.of(0.0);
	public static final Angle kRetractPosition = Degrees.of(0.0);

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
						.withFeedbackRemoteSensorID(kEncoderID)
						.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
						.withSensorToMechanismRatio(1.0)
						.withRotorToSensorRatio(kGearRatio))
				.withMotionMagic(new MotionMagicConfigs()
						.withMotionMagicCruiseVelocity(kCruiseVelocity)
						.withMotionMagicAcceleration(kAcceleration));
	}

	// Encoder Config
	public static CanCoderConfig encoderConfig() {
		CanCoderConfig config = new CanCoderConfig();

		config.CanCoderId = IntakeConstants.kEncoderID;
		config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

		return config;
	}
}
