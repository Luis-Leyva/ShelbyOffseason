// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Rollers extends SubsystemBase {

	private OverTalonFX m_motor = new OverTalonFX(RollerConstants.motorConfig(), RollerConstants.kMotorID,
			RobotConstants.rio);

	private VoltageOut m_voltageOut = new VoltageOut(0.0)
			.withEnableFOC(true);

	private double m_targetVoltage = 0.0;

	/** Creates a new Hood. */
	public Rollers() {
	}

	public Command setVoltage(double voltage) {
		return run(() -> m_motor.setControl(m_voltageOut.withOutput(m_targetVoltage)))
				.beforeStarting(() -> m_targetVoltage = voltage);
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Rollers/TargetVoltage", m_targetVoltage);
		SmartDashboard.putNumber("Rollers/MotorVoltage", m_motor.getMotorVoltage().getValueAsDouble());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
