package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

	private final XboxController driverController = new XboxController(0);
	private final XboxController operatorController = new XboxController(1);
	private final Drivetrain m_swerve = new Drivetrain();
	private final Arm m_arm = new Arm(18, 19, 21, 20);// CANId for Arm motors
	// private final LEDs m_leds = new LEDs();
	// Imposes a rate limit on how quickly the robot can change speed or direction,
	// to make joystick input more smooth
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
	public double slowRate = 1;
	public double driveTime;
	int m_autoStep;

	@Override
	public void robotInit() {
		
	}

	@Override
	public void disabledInit(){
		m_arm.setCoast(true);
		
	}

	@Override
	public void autonomousInit() {
		m_arm.setCoast(false);
		m_swerve.resetGyro();
		m_autoStep = 0;
	}

	@Override
	public void autonomousPeriodic() {
		scorePreLoaded();
	}

	@Override
	public void autonomousExit() {
		m_arm.setLiftSpeed(0.0);
	}

	@Override
	public void testInit() {
		m_arm.setCoast(false);
	}

	@Override
	public void testPeriodic() {
		scorePreLoaded();
	}

	@Override
	public void teleopInit() {
		m_arm.setCoast(false);
	}

	@Override
	public void teleopPeriodic() {
		updateTeleopDrive(false);
		updateTeleopArm();
		updateTeleopLeds();
	}

	private void updateTeleopDrive(boolean fieldRelative) {

		if (driverController.getRightBumper()){
			slowRate = .1;
		}
		else {
			slowRate = 1;
		}
		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final var xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftY()*slowRate, 0.05))
				* Drivetrain.kMaxSpeed;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftX()*slowRate, 0.05))
				* Drivetrain.kMaxSpeed;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getRightX()*slowRate, 0.10))
				* Drivetrain.kMaxAngularSpeed;

		m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);

	}

	private void updateTeleopArm() {

		// Button Control
		if ( operatorController.getBackButton() ){
			// Stow the Arm
			if ( m_arm.setExtension(0) ) {
				m_arm.setLiftAngle(66);
			}
		} else if ( operatorController.getStartButton() ){
			if ( m_arm.setExtension(0) ) {
				m_arm.setLiftAngle(0);
			}
		} else if ( operatorController.getAButton() ) {
			// Cone in Middle
			if ( m_arm.setLiftAngle(44) ) {
				m_arm.setExtension(-12);
			}
		} else if ( operatorController.getYButton() ) {
			// Cone on Top
			if ( m_arm.setLiftAngle(44) ) {
				m_arm.setExtension(-31);
			}
		} else if ( operatorController.getBButton() ) {
			// Cube on Middle
			if ( m_arm.setLiftAngle(36) ) {
				m_arm.setExtension(-1);
			}
		} else if ( operatorController.getXButton() ) {
			// Cube on Top
			if ( m_arm.setLiftAngle(39) ) {
				m_arm.setExtension(-19);
			}
		} else {
			// Manual Control
			m_arm.setExtensionSpeed(operatorController.getLeftY());
			m_arm.setLiftSpeed(-operatorController.getRightY());
		}

		if ( operatorController.getLeftBumper()) {
			m_arm.setIntake(0.3);
		} else if ( operatorController.getRightBumper() ){
			m_arm.setIntake(-0.3);
		} else {
			m_arm.setIntake(0);
		}

	}

	private void updateTeleopLeds() {
		// check if arm is moving
		double liftMotorSpeed = m_arm.getLiftSpeed();
		double extendMotorSpeed = m_arm.getExtensionSpeed();
		if ((liftMotorSpeed != 0) || (extendMotorSpeed != 0)) {
			boolean moving = true;
		} else {
			boolean moving = false;
		}

		// lights control
		if (operatorController.getLeftBumper()) {
			// m_leds.blink();
		}
	}
	
    private void scorePreLoaded() {

		if ( m_autoStep == 0 ) {
			m_autoStep = m_arm.setLiftAngle(45) ? 1 : 0;
		} else if (m_autoStep == 1) {
			m_autoStep = m_arm.setExtension(-31) ? 2 : 1;
		} else if ( m_autoStep == 2 ) {
			m_arm.setIntake(0.3);
			Timer.delay(1);
			m_arm.setIntake(0);
			m_autoStep = 3;
		} else if ( m_autoStep == 3 ) {
			m_autoStep = m_arm.setExtension(-0.5) ? 4 : 3;
		} else if (m_autoStep == 4) {
			m_autoStep = m_arm.setLiftAngle(67) ? 5 : 4;
			driveTime = Timer.getFPGATimestamp();
		} else if ( m_autoStep == 5 ) {
			m_swerve.drive(-1, 0, 0, false);
			double elapsedTime = Timer.getFPGATimestamp() - driveTime;
			if ( elapsedTime > 3 ) {
				m_autoStep = 6;
			}
		} else if ( m_autoStep == 6) {
			m_swerve.drive(0, 0, 0, false);
		}
    }

}