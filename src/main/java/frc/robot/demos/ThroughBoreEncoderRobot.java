package frc.robot.demos;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** REV Through-Bore Encoder demo robot */
public class ThroughBoreEncoderRobot extends CommandBaseRobot
{
  // Absolute readout uses (white, red, black) into DI
  private DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(3));
  
  @Override
  public void robotInit()
  {
    super.robotInit();
    encoder.reset();
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    // Encoder returns value 0...1, scale to 0..360 degrees
    SmartDashboard.putNumber("Position", encoder.getAbsolutePosition() * 360.0);
  }
}
