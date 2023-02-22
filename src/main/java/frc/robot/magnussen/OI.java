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

  public static void reset()
  {
    SwerveOI.reset();

    // Clear memory of past button presses
    for (int i=1; i<=buttons.getButtonCount(); ++i)
      buttons.getRawButtonPressed(i);
  }

  public static boolean selectUphillMode()
  {
    return joystick.getLeftBumperPressed();
  }

  public static boolean selectNearNodeMode()
  {
    return buttons.getRawButtonPressed(0); // TODO: Which button??
  }
}
