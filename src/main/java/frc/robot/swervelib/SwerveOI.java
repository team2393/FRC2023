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
  /** Use correct stick setup, or the wrong one? */
  private static final boolean CORRECT = false;

  /** Maximum swerve speed sent from joystick input */
  public static double MAX_METERS_PER_SEC = 3.0;

  /** Maximum rotational speed sent from joystick input */
  public static double MAX_RAD_PER_SEC = Math.toRadians(180);

  public static final XboxController joystick = new XboxController(0);

  // Limit joystick slew, go 0 to 1 in 1/4 second
  private static final SlewRateLimiter x_throttle = new SlewRateLimiter(8.0);
  private static final SlewRateLimiter y_throttle = new SlewRateLimiter(8.0);
  private static final SlewRateLimiter rot_throttle = new SlewRateLimiter(16.0);

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
    double stick = CORRECT ? joystick.getRightY() : joystick.getLeftY();
    stick = MathUtil.applyDeadband(stick, 0.1);
    stick = x_throttle.calculate(stick);
    return -MAX_METERS_PER_SEC * stick;
  }

  /** @return Left/right speed [m/s] */
  public static double getLeftSpeed()
  {
    double stick = CORRECT ? joystick.getRightX() : joystick.getLeftX();
    stick = MathUtil.applyDeadband(stick, 0.1);
    stick = y_throttle.calculate(stick);
    return -MAX_METERS_PER_SEC * stick;
  }

  /** @return Rotational speed, counter-clockwise [rad/s] */
  public static double getRotationSpeed()
  {
    double stick = CORRECT ? joystick.getLeftX() : joystick.getRightX();
    stick = MathUtil.applyDeadband(stick, 0.1);
    // Square output (keeping sign) for more sensitive center moves
    stick *= Math.abs(stick);
    stick = rot_throttle.calculate(stick);
    return -MAX_RAD_PER_SEC * stick;
  }

  /** @return Value -1..1 based on both triggers */
  public static double getCombinedTriggerValue()
  {
     return joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis();
  }

  public static boolean selectAbsoluteMode()
  {
    return joystick.getLeftBumperPressed();
  }

  public static boolean selectRelativeMode()
  {
    return joystick.getRightBumperPressed();
  }

  public static boolean selectFixedForward()
  {
    return joystick.getPOV() == 0;
  }

  public static boolean selectFixedBack()
  {
    return joystick.getPOV() == 180;
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
