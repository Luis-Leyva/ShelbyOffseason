// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

/**
 * Robot-wide numerical and boolean constants.
 */
public final class Constants {
	private Constants() {
	}

	/** CAN buses. Named to match the C++ robotConstants namespace. */
	public static final class RobotConstants {
		private RobotConstants() {
		}

		/** The roboRIO's own CAN bus. */
		public static final CANBus rio = new CANBus("rio");

		/** The CANivore carrying the drivetrain. */
		public static final CANBus overCANivore = new CANBus("OverCANivore");
	}
}
