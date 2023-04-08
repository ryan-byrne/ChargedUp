package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// DriveTrain handles control and movement of swerveDrive
public class Drivetrain {
		//initialize instance variables
	public static final double kMaxSpeed = 3.0; 
	public static final double kMaxAngularSpeed = Math.PI; 

	private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
	private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
	private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
	private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

	private final ShuffleboardTab m_driveTab;
	
	//Four swerveModule objects, one for each wheel of the robot
	// Module A
	private final SwerveModule m_frontLeft = new SwerveModule(10, 11);
	// Module B
	private final SwerveModule m_frontRight = new SwerveModule(12, 13);
	// Module C
	private final SwerveModule m_backLeft = new SwerveModule(14, 15);
	// Module D
	private final SwerveModule m_backRight = new SwerveModule(16, 17);
	// Robot's orientation in space
	public final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

	//Calculates wheel speeds and directions
	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	//tracks robot's position and orientation on field
	private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
			m_kinematics,
			m_gyro.getRotation2d(),
			new SwerveModulePosition[] {
					m_frontLeft.getPosition(),
					m_frontRight.getPosition(),
					m_backLeft.getPosition(),
					m_backRight.getPosition()
			});
		//constructor 
	public Drivetrain() {

		m_gyro.calibrate();

		m_driveTab = Shuffleboard.getTab("Drivetrain");
		m_driveTab.add("Gyro", m_gyro);
		m_driveTab.add("Front Left", m_frontLeft);
		//.withWidget(BuiltInWidgets.kDial)
		//.withProperties(Map.of(0, 2*))
		m_driveTab.add("Front Right", m_frontRight);
		m_driveTab.add("Back Left", m_backLeft);
		m_driveTab.add("Back Right", m_backRight);

	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates = m_kinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);


	}

	public void resetGyro(){
		m_gyro.reset();
	}

	public int getRelativeRotation(double rotation) {
		// if int returned is -1, robot rotation is less than desired rotation
		// if int returned is 1, robot rotation is more than desired rotation
		// if int returned is 0, robot rotation is desired rotation
		double BUFFER = 0.5;

		if ( m_gyro.getRotation2d().getDegrees() > rotation + BUFFER) {
			return 1;
		} else if ( m_gyro.getRotation2d().getDegrees() < rotation - BUFFER ) {
			return -1;
		} else if ( m_gyro.getRotation2d().getDegrees() > rotation - BUFFER || m_gyro.getRotation2d().getDegrees() < rotation + BUFFER){
			return 0;
		}
		return 0;
	}

	public void setPosition(int xPosition, int yPosition) {

	}

	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		m_odometry.update(
				m_gyro.getRotation2d(),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(),
						m_frontRight.getPosition(),
						m_backLeft.getPosition(),
						m_backRight.getPosition()
				});
	}
}
