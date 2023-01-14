package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.parts.RotationEncoder;
import frc.robot.parts.SparkMini;

/** Robot for testing drive motors */
public class RotationDemoRobot extends CommandBaseRobot
{
  XboxController joystick = new XboxController(0);
  SparkMini motor = new SparkMini(0);
  RotationEncoder angle = new RotationEncoder(0, 343);

  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.setDefaultNumber("P", .05);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();   
    SmartDashboard.putNumber("Raw Heading", angle.getRawHeading());
    SmartDashboard.putNumber("Adjusted Heading", angle.getHeading().getDegrees());
  } 

  @Override
  public void teleopPeriodic()
  {
    motor.set(joystick.getLeftX() * .2);
  }

  @Override
  public void autonomousPeriodic()
  {
    double currentAngle = angle.getHeading().getDegrees();
    double desiredAngle = 45;

    // Compute error as + or - 180
    double error = Math.IEEEremainder(desiredAngle - currentAngle, 360);
    motor.setVoltage(error * SmartDashboard.getNumber("P", 0));
  }
}
