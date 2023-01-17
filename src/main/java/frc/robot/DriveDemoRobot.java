package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Robot for testing drive motors */
public class DriveDemoRobot extends CommandBaseRobot
{
  final int TICKS_PER_METER = 41800;
  XboxController joystick = new XboxController(0);
  WPI_TalonFX motor = new WPI_TalonFX(1);

  @Override
  public void robotInit()
  {
    super.robotInit();
    // Allow manually moving the robot by disabling the default "brake" mode
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Coast);
  }

  /** @return Position in meters */
  double getPosition()
  {
    return motor.getSelectedSensorPosition() / TICKS_PER_METER;
  }

  /** @return Speed in meters per second */
  double getSpeed()
  {
    // Convert speed in ticks per 0.1 second to m/s
    return motor.getSelectedSensorVelocity() * 10.0 / TICKS_PER_METER;
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putNumber("Ticks", motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Position", getPosition());
    // TODO Display speed
  }

  @Override
  public void teleopPeriodic()
  {
    motor.setVoltage(joystick.getLeftY() * -10);
  }

  @Override
  public void autonomousPeriodic()
  {
    // TODO Run motor at a certain desired speed
  }
}
