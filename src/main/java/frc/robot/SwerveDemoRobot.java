package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.drivetrain.swerve.SwerveModule;

public class SwerveDemoRobot extends CommandBaseRobot
{
  SwerveModule module0 = new SwerveModule(0, -18);
  SwerveModule module1 = new SwerveModule(1, 90);
  SwerveModule module2 = new SwerveModule(2, -161);
  SwerveModule module3 = new SwerveModule(3, -104);
  XboxController joystick = new XboxController(0);




  @Override
  public void teleopPeriodic()
  {
    super.teleopPeriodic();
    double jx = -joystick.getRightY();
    double jy = -joystick.getRightX();
    double speed = Math.sqrt(jx*jx + jy*jy);
    double angle = Math.toDegrees(Math.atan2(jy, jx));
    module0.setSwerveModule(angle, speed);
    module1.setSwerveModule(angle, speed);
    module2.setSwerveModule(angle, speed);
    module3.setSwerveModule(angle, speed);
  }
}
