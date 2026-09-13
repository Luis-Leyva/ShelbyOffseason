// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.overture.lib.sensors.OverCANCoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Intake extends SubsystemBase {

	// Devices
	private OverTalonFX m_motor = new OverTalonFX(IntakeConstants.motorConfig(), IntakeConstants.kMotorID,
			RobotConstants.rio);
	private OverCANCoder m_encoder = new OverCANCoder(IntakeConstants.encoderConfig(), RobotConstants.rio);

	// Motor Outputs
	private MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(Degrees.of(0.0))
			.withEnableFOC(true)
			.withSlot(0);

	private Angle m_targetAngle = Degrees.of(0.0);

	public Intake() {
	}

	public Angle getError() {
		return m_targetAngle.minus(getEncoderAbsolutePosition());
	}

	public boolean isAtTarget() {
		return getError().abs(Degrees) < IntakeConstants.kAngleTolerance.in(Degrees);
	}

	public Command setAngle(Angle angle) {
		return run(() -> m_motor.setControl(m_motionMagic.withPosition(m_targetAngle)))
				.beforeStarting(() -> m_targetAngle = angle)
				.until((() -> isAtTarget()));
	}

	public Angle getMotorPosition() {
		return m_motor.getPosition().getValue();
	}

	public Angle getEncoderAbsolutePosition() {
		return m_encoder.getAbsolutePosition().getValue();
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Intake/MotorAngle", getMotorPosition().in(Degrees));
		SmartDashboard.putNumber("Intake/Error", getError().abs(Degrees));
		SmartDashboard.putBoolean("Intake/AtTarget", isAtTarget());
		SmartDashboard.putNumber("Intake/EncoderAngle", getEncoderAbsolutePosition().in(Degrees));
		SmartDashboard.putNumber("Intake/TargetAngle", m_targetAngle.in(Degrees));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
