// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import edu.wpi.first.wpilibj.XboxController;

/** Operator Interface */
public class OI
{
  private static final double MAX_METERS_PER_SEC = 0.6;

  private static final double MAX_RAD_PER_SEC = 0.3*Math.PI;

  public static final XboxController joystick = new XboxController(0);

  public static void reset()
  {
    // Clear memory of past button presses
    for (int i=1; i<=joystick.getButtonCount(); ++i)
      joystick.getRawButtonPressed(i);
  }

  /** @return Forward/backwards speed [m/s] */
  public static double getForwardSpeed()
  {
    return -MAX_METERS_PER_SEC * joystick.getRightY();
  }

  /** @return Left/right speed [m/s] */
  public static double getLeftSpeed()
  {
    return -MAX_METERS_PER_SEC * joystick.getRightX();
  }

  /** @return Rotational speed, counter-clockwise [rad/s] */
  public static double getRotationSpeed()
  {
    return -MAX_RAD_PER_SEC * joystick.getLeftX();
  }

  public static boolean selectAbsoluteMode()
  {
    return joystick.getLeftBumperPressed();
  }

  public static boolean selectRelativeMode()
  {
    return joystick.getRightBumperPressed();
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
