// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import frc.robot.CommandBaseRobot;
import frc.robot.magnussen.OI;
import frc.robot.magnussen.Pneumatics;

/** Test Arm/lift/grabber/intake coordinator */
public class CharmTestRobot extends CommandBaseRobot
{
  private Pneumatics pneumatics = new Pneumatics();
  private Charm coordinator = new Charm();

  @Override
  public void teleopInit()
  {
    OI.reset();
  }

  @Override
  public void teleopPeriodic()
  {
    if (OI.selectSubstationIntakeMode())
      coordinator.intakeFromSubstation(OI.selectCubeIntake());

    if (OI.joystick.getRightBumperPressed())
      coordinator.getDefaultCommand().schedule();

    if (OI.selectNearNodeMode())
      coordinator.near();

    if (OI.selectMiddleNodeMode())
      coordinator.mid();

    if (OI.selectFarNodeMode())
      coordinator.far();

    if (OI.ejectGamepiece())
      coordinator.eject();
  }
}
