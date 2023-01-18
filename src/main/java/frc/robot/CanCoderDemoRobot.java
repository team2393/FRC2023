package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** CANcoder demo robot */
public class CanCoderDemoRobot extends CommandBaseRobot
{
  private final CANCoder encoder = new CANCoder(0);

  @Override
  public void robotInit()
  {
    super.robotInit();

    encoder.configFactoryDefault();
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();

    SmartDashboard.putNumber("Angle", encoder.getAbsolutePosition());
  }
}
