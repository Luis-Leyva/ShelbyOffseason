// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.overture.lib.sensors.OverCANCoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Intake extends SubsystemBase {
	/** Creates a new Shooter. */

	// Devices
	private OverTalonFX testMotor = new OverTalonFX(IntakeConstants.motorConfig(), IntakeConstants.SHOOTER_MOTOR_ID,
			RobotConstants.rio);
	private OverCANCoder testEncoder = new OverCANCoder(IntakeConstants.encoderConfig(), RobotConstants.rio);

	// Motor Outputs
	private VoltageOut shooterVoltage = new VoltageOut(0.0);
	private PositionVoltage shooterPositionVoltage = new PositionVoltage(0.0);

	private Angle targetPosition = Degrees.of(0.0);

	public Intake() {

	}

	public Command setVoltage(double voltage) {
		return runOnce(() -> {
			testMotor.setControl(shooterVoltage.withEnableFOC(false).withOutput(voltage));
		});
	}

	public Command setPosition(Angle position) {
		return runOnce(() -> {
			targetPosition = position;
			testMotor.setControl(shooterPositionVoltage.withEnableFOC(false).withPosition(targetPosition).withSlot(0)
					.withFeedForward(0.1));
		}).until((() -> isAtTarget()));
	}

	public Angle getError() {
		return targetPosition.minus(getEncoderAbsolutePosition());
	}

	public boolean isAtTarget() {
		return getError().abs(Degrees) < IntakeConstants.POSITION_TOLERANCE.in(Degrees);
	}

	public double getVelocity() {
		return testMotor.getRotorVelocity().getValueAsDouble();
	}

	public Angle getMotorPosition() {
		return testMotor.getPosition().getValue();
	}

	public Angle getEncoderAbsolutePosition() {
		return testEncoder.getAbsolutePosition().getValue();
	}

	public void updateTelemetry() {
		// SmartDashboard.putNumber("Shooter Velocity", getVelocity());
		SmartDashboard.putNumber("Motor/Position/Degrees", getMotorPosition().in(Degrees));
		SmartDashboard.putNumber("Motor/Position/Error/Degrees", getError().abs(Degrees));
		SmartDashboard.putBoolean("Motor/Position/AtTarget", isAtTarget());
		SmartDashboard.putNumber("Encoder/AbsolutePosition/Degrees", getEncoderAbsolutePosition().in(Degrees));
		SmartDashboard.putNumber("Target/Position/Degrees", targetPosition.in(Degrees));
		SmartDashboard.putNumber("Test", IntakeConstants.POSITION_TOLERANCE.baseUnitMagnitude());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
