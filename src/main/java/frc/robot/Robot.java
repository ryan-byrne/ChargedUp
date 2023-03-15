package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

	private final Joystick driverJoystick = new Joystick(0);

	// private final XboxController driverController = new XboxController(0);
	private final XboxController operatorController = new XboxController(1);
	private final Drivetrain m_swerve = new Drivetrain();
	private final Arm m_arm = new Arm(18, 19, 20, 0, 1, 2, 3);// CANId for Arm motors
	// private final LEDs m_leds = new LEDs();
	// Imposes a rate limit on how quickly the robot can change speed or direction,
	// to make joystick input more smooth
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

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
		//updateArm();
		//updateDrive(true);
	}

	@Override
	public void teleopPeriodic() {
		updateDrive(true);
		updateArm();
		updateLeds();
	}

	private void updateDrive(boolean fieldRelative) {

		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverJoystick.getY(), 0.02))
				* Drivetrain.kMaxSpeed;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverJoystick.getX(), 0.02))
				* Drivetrain.kMaxSpeed;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(driverJoystick.getTwist(), 0.02))
				* Drivetrain.kMaxAngularSpeed;

		m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);

	}

	private void updateArm() {

		// Button ontrol

		// Manual Control
		m_arm.setExtensionSpeed(operatorController.getLeftY());
		m_arm.setLiftSpeed(-operatorController.getRightY());

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

}