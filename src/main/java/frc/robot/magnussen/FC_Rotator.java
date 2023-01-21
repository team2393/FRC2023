package frc.robot.magnussen;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.swervelib.Rotator;

public class FC_Rotator extends Rotator
{

  private final WPI_TalonFX motor;
  private final CANCoder sensor;

  public FC_Rotator(int index, double offset)
{
  super(index, offset);
  motor = new WPI_TalonFX(index+1, "CANivore2393");
  motor.configFactoryDefault();
  motor.setNeutralMode(NeutralMode.Coast);
  sensor = new CANCoder(index+1, "CANivore2393");
  sensor.configFactoryDefault();
  sensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
  
}

  @Override
  public double getRawDegrees() 
  {
    return sensor.getAbsolutePosition();
  }

  @Override
  public void setVoltage(double voltage) 
  {
    motor.setVoltage(voltage);
    
  }
  
}
