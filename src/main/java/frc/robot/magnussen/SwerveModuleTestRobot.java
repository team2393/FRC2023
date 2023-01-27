package frc.robot.magnussen;

import frc.robot.CommandBaseRobot;

public class SwerveModuleTestRobot extends CommandBaseRobot
{
  FC_Rotator rotator = new FC_Rotator(0, 13);

  @Override
  public void teleopPeriodic()
  {
      rotator.setVoltage(OI.joystick.getLeftX()*-5);
  }

  @Override
  public void autonomousPeriodic()
  {
    // Cycle between 0 and 90 degrees every 2 seconds
    long cycle = (System.currentTimeMillis() / 2000) % 2;
    double angle = cycle * 90.0;
    rotator.setAngle(angle);
  }
}
