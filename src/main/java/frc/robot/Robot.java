package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import java.util.Timer;

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
	private final Timer m_timer = new Timer();
	public double currentTime;

	@Override
	public void robotInit() {
		
	}

	@Override
	public void autonomousInit() {

	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
		updateArm();
		updateDrive(false);
	}

	@Override
	public void teleopInit() {
		m_swerve.resetGyro();
	}

	@Override
	public void teleopPeriodic() {

		updateDrive(true);
		updateArm();
		updateLeds();
	}

	private void updateDrive(boolean fieldRelative) {

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
		final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getRightX()*slowRate, 0.05))
				* Drivetrain.kMaxAngularSpeed;

		m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);

	}

	private void updateArm() {

		// Button Control


		// Manual Control
		m_arm.setExtensionSpeed(operatorController.getLeftY());
		m_arm.setLiftSpeed(-operatorController.getRightY());

		if ( operatorController.getRightBumper() ) {
			m_arm.setIntake(-1);
		} else if ( operatorController.getLeftBumper() ){
			m_arm.setIntake(0.3);
		} else if ( operatorController.getRightTriggerAxis() > .5) {
			m_arm.setIntake(-.5);
		} else {
			m_arm.setIntake(0);
		}

	}

	private void updateLeds() {
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
	private void auto(){
		// Score cone
		
	}

}