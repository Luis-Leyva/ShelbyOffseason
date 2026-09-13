// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Hood extends SubsystemBase {

	private OverTalonFX m_motor = new OverTalonFX(HoodConstants.motorConfig(), HoodConstants.kMotorID,
			RobotConstants.rio);

	private MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(Degrees.of(0.0))
			.withEnableFOC(true)
			.withSlot(0);

	private Angle m_targetAngle = Degrees.of(0.0);

	/** Creates a new Hood. */
	public Hood() {
	}

	public Angle getError() {
		return m_targetAngle.minus(m_motor.getPosition().getValue());
	}

	public boolean isAtTarget() {
		return getError().abs(Degrees) < HoodConstants.kAngleTolerance.in(Degrees);
	}

	public Command setAngle(Angle angle) {
		return run(() -> m_motor.setControl(m_motionMagic.withPosition(angle)))
				.beforeStarting(() -> m_targetAngle = angle)
				.until(() -> isAtTarget());
	}

	public Angle getAngle() {
		return m_motor.getPosition().getValue();
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Hood/CurrentAngle", getAngle().in(Degrees));
		SmartDashboard.putNumber("Hood/TargetAngle", m_targetAngle.in(Degrees));
		SmartDashboard.putBoolean("Hood/AtTarget", isAtTarget());
		SmartDashboard.putNumber("Hood/Error", m_motor.getClosedLoopError().getValueAsDouble() * 360.0);
		SmartDashboard.putNumber("Hood/TargetAngle", m_motor.getPosition().getValue().in(Degrees));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
