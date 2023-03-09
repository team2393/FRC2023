package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** - Set offset and check angles so that 'fully out, horizontal' equals zero degrees
 *  - 'Vertical' equals 90 degrees
 *  - Moving right stick 'up' moves intake 'in'
 *  - Moving left stick 'up' turns spinner to pull game piece 'in'
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
    SmartDashboard.setDefaultNumber("Setpoint1", 0.0);
    SmartDashboard.setDefaultNumber("Setpoint2", 0.0);
  }

  @Override
  public void teleopPeriodic()
  {
    // For initial tests, drive intake motor directly.
    // Tick 'up' should move 'in'
    double voltage = -5.0 * OI.joystick.getRightY();
    intake.setVoltage(voltage);
    SmartDashboard.putNumber("Intake Voltage", voltage);

    voltage = -5.0 * OI.joystick.getLeftY();
    intake.setSpinner(voltage);
    SmartDashboard.putNumber("Spinner Voltage", voltage);
  }

  @Override
  public void autonomousPeriodic()
  {
    double angle = (System.currentTimeMillis() / 5000) % 2 == 1
                 ? SmartDashboard.getNumber("Setpoint1", 0)
                 : SmartDashboard.getNumber("Setpoint2", 0);
    intake.setAngle(angle);
  }
}
