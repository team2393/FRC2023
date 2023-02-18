// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import frc.robot.CommandBaseRobot;

/** Fake lift/arm/... Robot
 *
 *  Used to simulate a robot where we control the lift/arm/intake
 */
public class LiftArmDummyRobot extends CommandBaseRobot
{
  private final TheGreatCoordinator tgc = new TheGreatCoordinator(true);

  @Override
  public void teleopPeriodic()
  {
    tgc.run();
  }
}
