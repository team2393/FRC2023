// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swervelib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

/** Operator Interface for swerving */
public class SwerveOI
{
  private static final double MAX_METERS_PER_SEC = 0.6;

  private static final double MAX_RAD_PER_SEC = Math.toRadians(90);

  public static final XboxController joystick = new XboxController(0);

  // Limit joystick slew, go 0 to 1 in 1/2 second
  private static final SlewRateLimiter x_throttle = new SlewRateLimiter(2.0);
  private static final SlewRateLimiter y_throttle = new SlewRateLimiter(2.0);
  private static final SlewRateLimiter rot_throttle = new SlewRateLimiter(2.0);

  public static void reset()
  {
    // Clear memory of past button presses
    for (int i=1; i<=joystick.getButtonCount(); ++i)
      joystick.getRawButtonPressed(i);
    
    x_throttle.reset(0.0);
    y_throttle.reset(0.0);
  }

  /** @return Forward/backwards speed [m/s] */
  public static double getForwardSpeed()
  {
    return -MAX_METERS_PER_SEC * MathUtil.applyDeadband(x_throttle.calculate(joystick.getRightY()), 0.1);
  }

  /** @return Left/right speed [m/s] */
  public static double getLeftSpeed()
  {
    return -MAX_METERS_PER_SEC * MathUtil.applyDeadband(y_throttle.calculate(joystick.getRightX()), 0.1);
  }

  /** @return Rotational speed, counter-clockwise [rad/s] */
  public static double getRotationSpeed()
  {
    return -MAX_RAD_PER_SEC * MathUtil.applyDeadband(rot_throttle.calculate(joystick.getLeftX()), 0.1);
  }

  public static boolean selectAbsoluteMode()
  {
    return joystick.getLeftBumperPressed();
  }

  public static boolean selectRelativeMode()
  {
    return joystick.getRightBumperPressed();
  }

  public static boolean selectFixedSpeed()
  {
    return joystick.getPOV() == 0;
  }

  public static boolean resetOrigin()
  { // Small right button
    return joystick.getStartButtonPressed();
  }

  public static boolean resetCenter()
  { // Small button next to POV
    return joystick.getBackButtonPressed();
  }

  public static boolean frontCenter()
  {
    return joystick.getYButtonPressed();
  }

  public static boolean rightCenter()
  {
    return joystick.getBButtonPressed();
  }

  public static boolean leftCenter()
  {
    return joystick.getXButtonPressed();
  }

  public static boolean backCenter()
  {
    return joystick.getAButtonPressed();
  }
}
