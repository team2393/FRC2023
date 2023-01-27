package frc.robot.magnussen;

import frc.robot.CommandBaseRobot;
import frc.robot.swervelib.SwerveModule;

public class SwerveModuleTestRobot extends CommandBaseRobot
{
  FC_Rotator rotator = new FC_Rotator(0, 13);

  @Override
  public void teleopPeriodic()
  {
      rotator.setVoltage(OI.joystick.getLeftX()*-5);
  }

  @Override
  public void autonomousPeriodic() {
    rotator.setAngle(0);
  }
}
