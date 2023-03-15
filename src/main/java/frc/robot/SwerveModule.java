package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
	//initialize instance variables
	private static final double kWheelRadius = 0.0508;
	private static final int kTurningEncoderResolution = 1024;

	private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
	private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; 

	private final WPI_TalonSRX m_turningMotor; //turn
	private final CANSparkMax m_drivingMotor; //forwards & backwards
	private final SparkMaxAnalogSensor m_drivingEncoder;

	private final String m_moduleId;
	
	private final double m_turnRatio;

	// Gains are for example purposes only - must be determined for your own robot!
	private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

	// Gains are for example purposes only - must be determined for your own robot!
	private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
			1,
			0,
			0,
			new TrapezoidProfile.Constraints(
					kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

	// Gains are for example purposes only - must be determined for your own robot!
	private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
	private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

	/**
	 * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
	 * and turning encoder.
	 *
	 * @param driveMotorChannel      PWM output for the drive motor.
	 * @param turningMotorChannel    PWM output for the turning motor.
	 * @param turningEncoderChannelA DIO input for the turning encoder channel A
	 * @param turningEncoderChannelB DIO input for the turning encoder channel B
	 */
	//constructor 
	public SwerveModule(
			String moduleId,
			int driveMotorCANId,
			int turningMotorCANId
			) {

		m_moduleId = moduleId;

		// Initialize Motors
		m_drivingMotor = new CANSparkMax(driveMotorCANId, CANSparkMaxLowLevel.MotorType.kBrushless);
		m_turningMotor = new WPI_TalonSRX(turningMotorCANId);
		// Initialize Encoders
		m_drivingEncoder = m_drivingMotor.getAnalog(Mode.kAbsolute);
		m_turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 50);
		
		// Set the distance per pulse for the drive encoder. We can simply use the
		// distance traveled for one rotation of the wheel divided by the encoder resolution
		m_drivingEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius);

		// Set the distance (in this case, angle) in radians per pulse for the turning
		// encoder.
		// This is the the angle through an entire rotation (2 * pi) divided by the
		// encoder resolution.
		m_turnRatio = 2*Math.PI / kTurningEncoderResolution;

		// Limit the PID Controller's input range between -pi and pi and set the input
		// to be continuous.
		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
	}

	public SwerveModuleState getState() {//returns current state of swerveModule (speed,angle)
		return new SwerveModuleState(
				m_drivingEncoder.getVelocity(), new Rotation2d(m_turningMotor.getSelectedSensorPosition() * m_turnRatio));
	}

	public SwerveModulePosition getPosition() {//returns current position of swerveModule(distance traveled, angle)
		return new SwerveModulePosition(
				m_drivingEncoder.getPosition(), new Rotation2d(m_turningMotor.getSelectedSensorPosition() * m_turnRatio));
	}

	public void setDesiredState(SwerveModuleState desiredState) {//desired State with speed and angle
		// Optimize the reference state to avoid spinning further than 90 degrees
		SwerveModuleState state = SwerveModuleState.optimize(desiredState,
				new Rotation2d(m_turningMotor.getSelectedSensorPosition() * m_turnRatio));

		// Calculate the drive output from the drive PID controller (velocity & rotation)
		final double driveOutput = m_drivePIDController.calculate(m_drivingEncoder.getVelocity(),
				state.speedMetersPerSecond);
		final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

		// Calculate the turning motor output from the turning PID controller.
		final double turnOutput = m_turningPIDController.calculate(m_turningMotor.getSelectedSensorPosition() * m_turnRatio,
				state.angle.getRadians());

		final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

		System.out.println(m_moduleId + ": " + state.angle.getDegrees());

		m_drivingMotor.setVoltage(driveOutput + driveFeedforward);
		m_turningMotor.setVoltage(turnOutput + turnFeedforward);
	}

	public double getDistance(){
		return m_drivingEncoder.getPosition();
	}
}