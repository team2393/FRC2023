package frc.robot.demos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** REV Through-Bore Encoder connected to SparkMax demo robot */
public class ThroughBoreEncoderOnSparkRobot extends CommandBaseRobot
{
  // Motor controller, presumably with NEO motor connected to it
  // via the 3 phase power and the motor sensor wire.
  // In addition, a REV ThroughBore is wired to the controller's data port
  // via a REV Absolute Encoder Adapter REV-11-3326 
  private CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder;
  
  @Override
  public void robotInit()
  {
    super.robotInit();
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    // Encoder returns value 0...1, scale to 0..360 degrees
    SmartDashboard.putNumber("Position", encoder.getPosition() * 360.0);
  }
}
