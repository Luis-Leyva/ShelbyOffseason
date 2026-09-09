// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.overture.lib.sensors.OverPigeon;
import com.overture.lib.subsystems.swerve.SwerveChassis;
import com.overture.lib.subsystems.swerve.SwerveModule;
import com.overture.lib.subsystems.swerve.SwerveModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.RobotConstants;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * Port of the C++ Chassis subsystem, the OvertureLib SwerveChassis subclass.
 *
 * <p>Two C++ overrides are gone and are not missing by accident. getDriveBaseRadius() went away when
 * PathPlanner started reading its RobotConfig from deploy/pathplanner/settings.json, and
 * getRotation3d() is no longer part of the SwerveBase contract.
 */
public class Chassis extends SwerveChassis {
  /** Drive reduction, shared by the real module config and the simulated drivetrain below. */
  private static final double kDriveGearRatio = 7.03;

  /** Steer reduction, likewise shared. */
  private static final double kTurnGearRatio = 287.0 / 11.0;

  private static final double kWheelRadiusInches = 2.0;
  private static final double kTrackXInches = 11.375;
  private static final double kTrackYInches = 10.375;

  private final OverPigeon pigeon = new OverPigeon(13, RobotConstants.overCANivore);

  private final SwerveModule frontLeftModule = new SwerveModule(frontLeftConfig());
  private final SwerveModule frontRightModule = new SwerveModule(frontRightConfig());
  private final SwerveModule backLeftModule = new SwerveModule(backLeftConfig());
  private final SwerveModule backRightModule = new SwerveModule(backRightConfig());

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d(
              Units.inchesToMeters(kTrackXInches), Units.inchesToMeters(kTrackYInches)),
          new Translation2d(
              Units.inchesToMeters(kTrackXInches), Units.inchesToMeters(-kTrackYInches)),
          new Translation2d(
              Units.inchesToMeters(-kTrackXInches), Units.inchesToMeters(kTrackYInches)),
          new Translation2d(
              Units.inchesToMeters(-kTrackXInches), Units.inchesToMeters(-kTrackYInches)));

  /**
   * Drive feedforward: ks volts, kv volts per m/s, ka volts per m/s^2.
   *
   * <p>The C++ declared four separate SimpleMotorFeedforward objects and then handed them to the
   * wrong modules - FrontRightConfig() took feedForwardFrontLeft, BackRightConfig() took
   * feedForwardFrontRight, and so on. All four held identical values so nothing misbehaved, but the
   * first person to tune one module would have tuned a different wheel. One factory removes the
   * possibility.
   *
   * @return the drive feedforward
   */
  private static SimpleMotorFeedforward driveFeedForward() {
    return new SimpleMotorFeedforward(0.0, 2.0879, 0.098433);
  }

  /** Builds the drivetrain and wires it into PathPlanner. */
  public Chassis() {
    super();
    configureSwerveBase();
    setAcceptingVisionMeasurements(true);
    resetHeading();
  }

  /**
   * Describes this drivetrain to the physics simulation.
   *
   * <p>These numbers are the ones RobotSim-Maple used for Shelby2, verified against
   * overture/sim/swerve/Constants.java: 61.235 kg, a Pigeon2, 30.5 x 31.5 inch bumpers, and the
   * same module translations as the kinematics above.
   *
   * <p>The gearing constants are the same ones the module configs use below. That is the entire
   * point: the previous arrangement kept them in a separate simulation repository, where a drive
   * ratio could sit at 6.03 against a robot geared 7.03 without anyone noticing.
   */
  @Override
  protected DriveTrainSimulationConfig getSimulationConfig() {
    return DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(61.235))
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(1),
                DCMotor.getFalcon500(1),
                kDriveGearRatio,
                kTurnGearRatio,
                Volts.of(0.01),
                Volts.of(0.01),
                Inches.of(kWheelRadiusInches),
                KilogramSquareMeters.of(0.03),
                // Conservative tyre friction. The COTS value was flagged as abnormal by the
                // simulation (~2.106); 0.9 is realistic for rubber on carpet.
                0.9))
        .withCustomModuleTranslations(
            new Translation2d[] {
              new Translation2d(Inches.of(kTrackXInches), Inches.of(kTrackYInches)),
              new Translation2d(Inches.of(kTrackXInches), Inches.of(-kTrackYInches)),
              new Translation2d(Inches.of(-kTrackXInches), Inches.of(kTrackYInches)),
              new Translation2d(Inches.of(-kTrackXInches), Inches.of(-kTrackYInches))
            })
        .withBumperSize(Inches.of(30.5), Inches.of(31.5));
  }

  @Override
  protected Pigeon2 getSimulationPigeon() {
    return pigeon;
  }

  @Override
  public double getMaxModuleSpeed() {
    return 4.541;
  }

  @Override
  protected SwerveModule getFrontLeftModule() {
    return frontLeftModule;
  }

  @Override
  protected SwerveModule getFrontRightModule() {
    return frontRightModule;
  }

  @Override
  protected SwerveModule getBackLeftModule() {
    return backLeftModule;
  }

  @Override
  protected SwerveModule getBackRightModule() {
    return backRightModule;
  }

  @Override
  protected SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public Rotation2d getRotation2d() {
    // Pigeon2.getYaw() is in DEGREES, unlike TalonFX.getPosition() and
    // CANcoder.getAbsolutePosition() which are rotations. The C++ built a Rotation2d straight from
    // GetYaw().GetValue(), which was correct there because the units library carried the
    // conversion. In Java that is a raw double, so use the built in conversion instead.
    return pigeon.getRotation2d();
  }

  @Override
  protected PIDConstants getTranslationPID() {
    return new PIDConstants(6.0, 0.0, 0.0);
  }

  @Override
  protected PIDConstants getRotationPID() {
    return new PIDConstants(6.0, 0.0, 0.0);
  }

  private static SwerveModuleConfig baseModuleConfig() {
    SwerveModuleConfig config = new SwerveModuleConfig(driveFeedForward());
    config.CanBus = RobotConstants.overCANivore;
    config.DriveGearRatio = kDriveGearRatio;
    config.TurnGearRatio = kTurnGearRatio;
    config.WheelDiameter = Units.inchesToMeters(4.0);
    config.DriveMotorConfig.CurrentLimit = 30.0;
    config.DriveMotorConfig.TriggerThreshold = 60.0;
    config.TurnMotorConfig.PIDConfigs.withKP(40).withKS(0.15);
    // Drive kP is deliberately left at zero. SwerveModule.setState now runs a closed velocity loop,
    // but at kP = 0 the feedforward alone is in charge, which reproduces the open loop behaviour
    // this robot ran at Worlds exactly. Raising it is the tuning that unlocks the closed loop; do
    // that on the robot with data, not blind in a port.
    config.TurnMotorConfig.Inverted = false;
    config.DriveMotorConfig.Inverted = false;
    return config;
  }

  // The C++ zeroed the CANcoder offsets off the roboRIO with #ifndef __FRC_ROBORIO__, because the
  // simulated encoders reported raw. SwerveModule now hands Phoenix the same offset through
  // CANcoderSimState.SensorOffset, so simulation runs the real calibration like the robot does and
  // the sim/real split is gone.

  private static SwerveModuleConfig frontRightConfig() {
    SwerveModuleConfig config = baseModuleConfig();
    config.DriveMotorConfig.MotorId = 2;
    config.TurnMotorConfig.MotorId = 1;
    config.EncoderConfig.CanCoderId = 9;
    config.EncoderConfig.Offset = 0.464111328125;
    config.ModuleName = "Front Right";
    return config;
  }

  private static SwerveModuleConfig backRightConfig() {
    SwerveModuleConfig config = baseModuleConfig();
    config.DriveMotorConfig.MotorId = 4;
    config.TurnMotorConfig.MotorId = 3;
    config.EncoderConfig.CanCoderId = 12;
    config.EncoderConfig.Offset = 0.352294921875;
    config.ModuleName = "Back Right";
    return config;
  }

  private static SwerveModuleConfig frontLeftConfig() {
    SwerveModuleConfig config = baseModuleConfig();
    config.DriveMotorConfig.MotorId = 6;
    config.TurnMotorConfig.MotorId = 5;
    config.EncoderConfig.CanCoderId = 10;
    config.EncoderConfig.Offset = -0.12255859375;
    config.ModuleName = "Front Left";
    return config;
  }

  private static SwerveModuleConfig backLeftConfig() {
    SwerveModuleConfig config = baseModuleConfig();
    config.DriveMotorConfig.MotorId = 8;
    config.TurnMotorConfig.MotorId = 7;
    config.EncoderConfig.CanCoderId = 11;
    // Only this module ramps. Carried over from the C++ as-is; the asymmetry looks unintentional
    // but it is real configuration, so changing it is a decision for the robot, not the port.
    config.DriveMotorConfig.OpenLoopRampRate = 0.01;
    config.EncoderConfig.Offset = -0.383056640625;
    config.ModuleName = "Back Left";
    return config;
  }
}
