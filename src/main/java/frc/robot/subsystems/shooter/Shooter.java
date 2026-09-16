// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Shooter extends SubsystemBase {

	private OverTalonFX m_motor = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.kMotorID,
			RobotConstants.rio);
	private OverTalonFX m_motor2 = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.kMotorID2,
			RobotConstants.rio);
	private OverTalonFX m_motor3 = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.kMotorID3,
			RobotConstants.rio);
	private OverTalonFX m_motor4 = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.kMotorID4,
			RobotConstants.rio);

	private MotionMagicVelocityVoltage m_velocityOut = new MotionMagicVelocityVoltage(0.0)
			.withEnableFOC(true);

	private AngularVelocity m_targetVelocity = RotationsPerSecond.of(0.0);

	/** Creates a new Hood. */
	public Shooter() {
		m_motor2.setFollow(ShooterConstants.kMotorID, false);
		m_motor3.setFollow(ShooterConstants.kMotorID, false);
		m_motor4.setFollow(ShooterConstants.kMotorID, false);
	}

	public Command setSpeed(AngularVelocity velocity) {
		return run(() -> m_motor.setControl(m_velocityOut.withVelocity(m_targetVelocity)))
				.beforeStarting(() -> m_targetVelocity = velocity);
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Shooter/TargetVelocity", m_targetVelocity.magnitude());
		SmartDashboard.putNumber("Shooter/MotorVoltage", m_motor.getVelocity().getValueAsDouble());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
