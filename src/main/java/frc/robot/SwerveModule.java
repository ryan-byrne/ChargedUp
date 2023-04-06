package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SwerveModule implements Sendable {
	// initialize instance variables
	private static final double kWheelRadius = 0.0508;
	private static final double m_driveConversion = 1 / 20.82;
	private static final int kTurningEncoderResolution = 1024;
	private static final double m_turnRatio = 2 * Math.PI / kTurningEncoderResolution;

	private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
	private static final double kModuleMaxAngularAcceleration = 2 * Math.PI;

	private final WPI_TalonSRX m_turningMotor;
	private final CANSparkMax m_drivingMotor;

	private final RelativeEncoder m_drivingEncoder;

	private SwerveModuleState m_swerveState = new SwerveModuleState();

	private final PIDController m_drivePIDController = new PIDController(2, 0, 0);

	private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
			5,
			5,
			0.1,
			new TrapezoidProfile.Constraints(
					kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

	private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 1);
	private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 1);

	/**
	 * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
	 * and turning encoder.
	 *
	 * @param driveMotorChannel      PWM output for the drive motor.
	 * @param turningMotorChannel    PWM output for the turning motor.
	 */
	// constructor
	public SwerveModule(
			int driveMotorCANId,
			int turningMotorCANId
			) {

		// Initialize Drive Motor
		m_drivingMotor = new CANSparkMax(driveMotorCANId, CANSparkMaxLowLevel.MotorType.kBrushless);
		m_drivingEncoder = m_drivingMotor.getEncoder();
		// Initialize Turning Motor
		m_turningMotor = new WPI_TalonSRX(turningMotorCANId);
		m_turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 50);
		// Limit the PID Controller's input range between -pi and pi and set the input
		// to be continuous.
		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

	}

	public SwerveModuleState getState() {// returns current state of swerveModule (speed,angle)
		return new SwerveModuleState(
				this.getDriveVelocity(), new Rotation2d(this.getTurnPosition()));
	}

	public SwerveModulePosition getPosition() {// returns current position of swerveModule(distance traveled, angle)
		return new SwerveModulePosition(
				this.getDrivePosition(), new Rotation2d(this.getTurnPosition()));
	}

	public void setDesiredState(SwerveModuleState desiredState) {// desired State with speed and angle
		// Optimize the reference state to avoid spinning further than 90 degrees
		m_swerveState = SwerveModuleState.optimize(desiredState,
				new Rotation2d(this.getTurnPosition()));

		// Calculate the drive output from the drive PID controller (velocity &
		// rotation)
		final double driveOutput = m_drivePIDController.calculate(this.getDriveVelocity(),
			m_swerveState.speedMetersPerSecond);
		final double driveFeedforward = m_driveFeedforward.calculate(m_swerveState.speedMetersPerSecond);

		// Calculate the turning motor output from the turning PID controller.
		final double turnOutput = m_turningPIDController.calculate(this.getTurnPosition(),
			m_swerveState.angle.getRadians());

		final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

		m_drivingMotor.setVoltage(driveOutput + driveFeedforward);
		m_turningMotor.setVoltage(turnOutput + turnFeedforward);
	}

	public double getTurnPosition() {
		return m_turningMotor.getSelectedSensorPosition() * m_turnRatio;
	}

	public double getTurnVelocity() {
		return m_turningMotor.getSelectedSensorVelocity() * m_turnRatio;
	}

	public double getTurnVoltage(){
		return m_turningMotor.getMotorOutputVoltage();
	}

	public double getDrivePosition() {
		return m_drivingEncoder.getPosition() * m_driveConversion;
	}

	public double getDriveVelocity() {
		return m_drivingEncoder.getVelocity() / 2000;
	}

	public double getDriveVoltage(){
		return m_drivingMotor.getAppliedOutput();
	}

	public double getDesiredDriveVelocity(){
		return m_swerveState.speedMetersPerSecond;
	}

	public double getDesiredTurnPosition(){
		return m_swerveState.angle.getRadians();
	}

	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Swerve Module");
		builder.addDoubleProperty("turn-position", this::getTurnPosition, null);
		builder.addDoubleProperty("drive-velocity", this::getDriveVelocity, null);
		//builder.addDoubleProperty("wheel-angle",thi, null);
	}
}