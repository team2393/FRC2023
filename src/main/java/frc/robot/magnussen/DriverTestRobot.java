package frc.robot.magnussen;

import frc.robot.CommandBaseRobot;

public class DriverTestRobot extends CommandBaseRobot
{
  FalconDriver driver = new FalconDriver(0);

  @Override
  public void disabledInit()
  {
    driver.brake(false);
  }

  @Override
  public void teleopInit()
  {
    driver.brake(true);
  }


  @Override
  public void teleopPeriodic()
  {
      driver.setVoltage(OI.joystick.getRightY()*-5);
  }

  @Override
  public void autonomousPeriodic()
  {
    // Cycle between 0 and 90 degrees every 2 seconds
    long cycle = (System.currentTimeMillis() / 5000) % 2;
    double speed;
    if (cycle == 0)
      speed = 0.5;
    else
       speed = 1.5;
    driver.setSpeed(speed);
  }
}
