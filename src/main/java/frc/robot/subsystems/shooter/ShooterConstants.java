package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

	// Device IDs
	public static final int kMotorID = 0;
	public static final int kMotorID2 = 0;
	public static final int kMotorID3 = 0;
	public static final int kMotorID4 = 0;

	// Speeds
	public static final AngularVelocity kStopSpeed = RotationsPerSecond.of(0.0);

	// Gear Ratio
	public static final double kGearRatio = (0.0 / 0.0);

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
								.withNeutralMode(NeutralModeValue.Coast))
				.withFeedback(new FeedbackConfigs()
						.withVelocityFilterTimeConstant(0.1)
						.withSensorToMechanismRatio(kGearRatio))
				.withSlot0(new Slot0Configs()
						.withKP(0.0)
						.withKV(0.0))
				.withMotionMagic(new MotionMagicConfigs()
						.withMotionMagicAcceleration(0.0)
						.withMotionMagicCruiseVelocity(0.0)
						.withMotionMagicJerk(0.0));

	}

}