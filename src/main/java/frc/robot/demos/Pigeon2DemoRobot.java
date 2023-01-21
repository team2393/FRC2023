package frc.robot.demos;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Pigeon2 demo robot */
public class Pigeon2DemoRobot extends CommandBaseRobot
{
  // Wasn't detected on CANivore
  // private final Pigeon2 encoder = new Pigeon2(0, "CANivore2393");
  private final Pigeon2 encoder = new Pigeon2(0, "rio");

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
    SmartDashboard.putNumber("Heading", encoder.getYaw());
  }
}
