// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    // driveWithJoystick(false);
    //m_swerve.updateOdometry();
  } 

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }
  double rot_radians = gyro.getAngle()*(Math.PI/180);
  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
        * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
        * Drivetrain.kMaxSpeed;

    // Convert the desired speeds according to the rotation of the robot
    ySpeed = ySpeed*Math.cos(rot_radians) + xSpeed*Math.sin(rot_radians);
    xSpeed = xSpeed*Math.cos(rot_radians) + ySpeed*Math.sin(rot_radians);
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
        * Drivetrain.kMaxAngularSpeed;

    //System.out.println("x:" + xSpeed + ", y:" + ySpeed + ", rotation:" + rot);
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
