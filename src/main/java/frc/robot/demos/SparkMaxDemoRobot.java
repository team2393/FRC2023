package frc.robot.demos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Demo of SparkMAX with mini 550 motor */
public class SparkMaxDemoRobot extends CommandBaseRobot
{
  CANSparkMax motor = new CANSparkMax(20, MotorType.kBrushless);
  XboxController controller = new XboxController(0);

  @Override
  public void robotInit()
  {
    motor.restoreFactoryDefaults();
  }

  @Override
  public void teleopPeriodic()
  {
    motor.setVoltage(controller.getLeftX());
    // Position in turns
    SmartDashboard.putNumber("velocity", motor.getEncoder().getVelocity());
    SmartDashboard.putNumber("position", motor.getEncoder().getPosition());
  }
}
