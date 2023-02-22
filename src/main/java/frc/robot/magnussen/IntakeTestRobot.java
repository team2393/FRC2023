package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** TODO
 *  - Set offset and check angles so that 'fully out, horizontal' equals zero degrees
 *  - 'Vertical' equals 90 degrees
 *  - Moving right stick 'up' moves intake 'in'
 *  - In autonomous, get intake to setpoint
 *  - Update teleop to toggle between 3 positions: 'out, 'up' to meet arm, 'stored'
 */
public class IntakeTestRobot extends CommandBaseRobot
{
  private final Intake intake = new Intake();

  @Override
  public void robotInit()
  {
    super.robotInit();
    OI.reset();
    SmartDashboard.setDefaultNumber("Intake Angle", 0.0);
    SmartDashboard.setDefaultNumber("Setpoint", 0.0);
  }

  @Override
  public void robotPeriodic()
  {
    SmartDashboard.putNumber("Intake Angle", intake.getAngle());
  }

  @Override
  public void teleopPeriodic()
  {
    // For initial tests, drive intake motor directly.
    // Tick 'up' should move 'in'
    double voltage = -5.0 * OI.joystick.getRightY();
    intake.setVoltage(voltage);
    SmartDashboard.putNumber("Intake Voltage", voltage);

    // TODO: Figure out how to alternate between different angles (0 degrees, 45 degrees, 90 degrees)
    // if (OI.joystick.getRightBumper() && intake.getAngle() < 90)
    // {
    //   intake.setAngle(intake.getAngle() + 45);
    // }
    // else if (OI.joystick.getLeftBumper() && intake.getAngle() > 0)
    // {
    //   intake.setAngle(intake.getAngle() - 45);
    // }
  }

  @Override
  public void autonomousPeriodic()
  {
    intake.setAngle(SmartDashboard.getNumber("Setpoint", 0));
  }
}
