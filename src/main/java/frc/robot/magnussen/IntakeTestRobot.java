package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

public class IntakeTestRobot extends CommandBaseRobot
{
  private final Intake intake = new Intake();

  @Override
  public void robotInit()
  {
    super.robotInit();
    OI.reset();
    SmartDashboard.setDefaultNumber("Intake Angle", 0.0);
  }

  @Override
  public void robotPeriodic()
  {
    SmartDashboard.putNumber("Intake Angle", intake.getAngle());
  }

  @Override
  public void teleopPeriodic() {
    // TODO: Figure out how to alternate between different angles (0 degrees, 45 degrees, 90 degrees)
    if (OI.joystick.getRightBumper())
    {
      intake.setAngle(intake.getAngle() + 45);
    }
    else if (OI.joystick.getLeftBumper())
    {
      intake.setAngle(intake.getAngle() - 45);
    }
  }
}
