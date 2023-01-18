package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** REV Through-Bore Encoder demo robot */
public class ThroughBoreEncoderRobot extends CommandBaseRobot
{
  // Absolute readout uses (white, red, black) into DI
  private DutyCycle encoder = new DutyCycle(new DigitalInput(1));
  
  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putNumber("Position", encoder.getHighTimeNanoseconds());
  }
}
