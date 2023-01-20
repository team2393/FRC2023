package frc.robot.demos;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** CANcoder demo robot */
public class CanCoderDemoRobot extends CommandBaseRobot
{
  private final CANCoder encoder = new CANCoder(0);

  @Override
  public void robotInit()
  {
    super.robotInit();

    encoder.configFactoryDefault();
    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();

    SmartDashboard.putNumber("Angle", encoder.getAbsolutePosition());
  }

  @Override
  public void teleopInit()
  {
    System.out.println(encoder.getLastUnitString());
  }
}
