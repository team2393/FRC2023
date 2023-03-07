// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import frc.robot.CommandBaseRobot;

/** Test Arm/lift/grabber/intake coordinator */
public class SecondAttemptTestRobot extends CommandBaseRobot
{
  private SecondAttempt coordinator = new SecondAttempt();

  @Override
  public void teleopInit()
  {
    OI.reset();
  }

  @Override
  public void teleopPeriodic()
  {
    // if (OI.joystick.getRawButtonPressed(6))
    //   coordinator.store();

    if (OI.selectIntakeMode())
      if (OI.selectCubeIntake())
        coordinator.intakeCube();
      else
        coordinator.intakeCone();

    if (OI.selectNearNodeMode())
      coordinator.near();

    if (OI.selectMiddleNodeMode())
      coordinator.mid();

    if (OI.selectFarNodeMode())
      coordinator.far();

    if (OI.ejectGamepiece())
      coordinator.eject();
    if (OI.clearJam())
      coordinator.clearJam();
  }
}
