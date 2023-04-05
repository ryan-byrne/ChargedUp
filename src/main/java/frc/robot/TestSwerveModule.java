package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class TestSwerveModule implements Sendable {

	private static final double m_driveConversion = 1 / 20.82;

	private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
	private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

	private final CANSparkMax m_driveMotor;
	private final WPI_TalonSRX m_turningMotor;

	private final RelativeEncoder m_driveEncoder;
	private final DutyCycleEncoder m_turningEncoder;

	private SwerveModuleState m_swerveState = new SwerveModuleState();

	// Gains are for example purposes only - must be determined for your own robot!
	private final PIDController m_drivePIDController = new PIDController(0.1, 0, 0);

	// Gains are for example purposes only - must be determined for your own robot!
	private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
			2,
			1,
			0,
			new TrapezoidProfile.Constraints(
					kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

	// Gains are for example purposes only - must be determined for your own robot!
	private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 1);
	private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 1);

	/**
	 * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
	 * and turning encoder.
	 *
	 * @param driveMotorChannel      PWM output for the drive motor.
	 * @param turningMotorChannel    PWM output for the turning motor.
	 * @param driveEncoderChannelA   DIO input for the drive encoder channel A
	 * @param driveEncoderChannelB   DIO input for the drive encoder channel B
	 * @param turningEncoderChannelA DIO input for the turning encoder channel A
	 * @param turningEncoderChannelB DIO input for the turning encoder channel B
	 */
	public TestSwerveModule(
			int driveMotorCanID,
			int turnMotorCanID,
			int turningEncoderChannel) {

		m_driveMotor = new CANSparkMax(driveMotorCanID, CANSparkMaxLowLevel.MotorType.kBrushless);
		m_driveEncoder = m_driveMotor.getEncoder();

		m_turningMotor = new WPI_TalonSRX(turnMotorCanID);
		m_turningEncoder = new DutyCycleEncoder(turningEncoderChannel);
		m_turningEncoder.setDistancePerRotation(2 * Math.PI);
		// Limit the PID Controller's input range between -pi and pi and set the input
		// to be continuous.
		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(
				this.getDrivePosition(), new Rotation2d(this.getTurnPosition()));
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
				this.getDriveVelocity(), new Rotation2d(this.getTurnPosition()));
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		m_swerveState = SwerveModuleState.optimize(desiredState,
				new Rotation2d(this.getDrivePosition()));

		// Calculate the drive output from the drive PID controller.
		final double driveOutput = m_drivePIDController.calculate(this.getDriveVelocity(),
		m_swerveState.speedMetersPerSecond);

		final double driveFeedforward = m_driveFeedforward.calculate(m_swerveState.speedMetersPerSecond);

		// Calculate the turning motor output from the turning PID controller.
		final double turnOutput = m_turningPIDController.calculate(this.getTurnPosition(),
		m_swerveState.angle.getRadians());

		final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

		m_driveMotor.setVoltage(driveOutput + driveFeedforward);
		m_turningMotor.setVoltage(turnOutput + turnFeedforward);
	}

	public double getTurnPosition() {
		return m_turningEncoder.getDistance();
	}

	public double getTurnVoltage(){
		return m_turningMotor.getMotorOutputVoltage();
	}

	public double getDesiredTurnPosition(){
		return m_swerveState.angle.getRadians();
	}

	public double getDrivePosition() {
		return m_driveEncoder.getPosition() * m_driveConversion;
	}

	public double getDriveVelocity() {
		return m_driveEncoder.getVelocity() / 2000;
	}

	public double getDriveVoltage(){
		return m_driveMotor.getAppliedOutput();
	}
	
	public double getDesiredDriveVelocity(){
		return m_swerveState.speedMetersPerSecond;
	}

	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Test Swerve");
		builder.addDoubleProperty("turn-position", this::getTurnPosition, null);
		builder.addDoubleProperty("turn-voltage", this::getTurnVoltage, null);
		builder.addDoubleProperty("turn-desired", this::getDesiredTurnPosition, null);
		builder.addDoubleProperty("drive-velocity", this::getDriveVelocity, null);
		builder.addDoubleProperty("drive-desired", this::getDesiredDriveVelocity, null);
		builder.addDoubleProperty("drive-voltage", this::getDriveVoltage, null);
	}
}