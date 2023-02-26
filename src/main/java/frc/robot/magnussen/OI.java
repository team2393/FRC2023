// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.swervelib.SwerveOI;

/** Operator interface 
 *
 *  Extends the SwerveOI for basic driving with
 *  operator interface elements for lift etc,
 *  using both the joystick and a button board
 */
public class OI extends SwerveOI
{
  public static final GenericHID buttons = new GenericHID(1);

  /** Clear memory of past button presses */
  public static void reset()
  {
    SwerveOI.reset();

    for (int i=1; i<=buttons.getButtonCount(); ++i)
      buttons.getRawButtonPressed(i);
  }

  /** @return Select cube? Otherwise: Cone */
  public static boolean selectCubeIntake()
  {
    return buttons.getRawButton(1);
  }

  public static boolean selectNormalDriveMode()
  {
    return TEST ? joystick.getRightBumperPressed()
                : buttons.getRawButtonPressed(7);
  }

  public static boolean selectUphillMode()
  {
    return TEST ? joystick.getLeftBumperPressed()
                : buttons.getRawButtonPressed(3);
  }

  public static boolean selectIntakeMode()
  {
    return TEST ? joystick.getAButtonPressed()
                : buttons.getRawButtonPressed(9); 
  }

  public static boolean ejectGamepiece()
  {
    return false; // TODO 3 Pick a button 
  }

  public static boolean selectNearNodeMode()
  {
    return TEST ? joystick.getXButtonPressed()
                : buttons.getRawButtonPressed(4); 
  }

  public static boolean selectMiddleNodeMode()
  {
    return TEST ? joystick.getYButtonPressed()
                : buttons.getRawButtonPressed(5); 
  }

  public static boolean selectFarNodeMode()
  {
    return TEST ? joystick.getBButtonPressed()
                : buttons.getRawButtonPressed(8); 
  }
}
