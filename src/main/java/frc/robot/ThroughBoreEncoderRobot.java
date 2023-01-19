package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** REV Through-Bore Encoder demo robot */
public class ThroughBoreEncoderRobot extends CommandBaseRobot
{
  // Absolute readout uses (white, red, black) into DI
  private DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(1));
  
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
