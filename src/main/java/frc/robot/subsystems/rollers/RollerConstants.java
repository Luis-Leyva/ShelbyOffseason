package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RollerConstants {

	// Device IDs
	public static final int kMotorID = 0;

	// Voltages
	public static final double kStopVoltage = 0.0;
	public static final double kIntakinVoltage = 0.0;
	public static final double kShootVoltage = 0.0;

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
								.withNeutralMode(NeutralModeValue.Coast));

	}

}