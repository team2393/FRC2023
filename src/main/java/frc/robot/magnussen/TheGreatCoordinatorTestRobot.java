// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import frc.robot.CommandBaseRobot;

/** Test TheGreatCoordinator */
public class TheGreatCoordinatorTestRobot extends CommandBaseRobot
{
  private final TheGreatCoordinator coordinator = new TheGreatCoordinator(true);

  @Override
  public void teleopInit()
  {
    OI.reset();
  }

  @Override
  public void teleopPeriodic()
  {
    coordinator.run();
  }
}
