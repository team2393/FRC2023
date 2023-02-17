// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Fake lift/arm/... Robot
 *
 *  Used to simulate a robot where we control the lift and arm
 */
public class LiftArmDummyRobot extends CommandBaseRobot
{
  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.putNumber("Lift Height", 0.0);
    SmartDashboard.putNumber("Arm Angle", -90.0);
    SmartDashboard.putNumber("Intake Angle", 90.0);
    SmartDashboard.putBoolean("Arm Extended", false);
  }

  private void adjust(String setting, double rate, double min, double max)
  {
    double value = SmartDashboard.getNumber(setting, 0.0);
    value += rate;
    SmartDashboard.putNumber(setting, MathUtil.clamp(value, min, max));
  }

  @Override
  public void teleopPeriodic()
  {
    adjust("Lift Height", -0.02*MathUtil.applyDeadband(OI.joystick.getRightY(), 0.1),    0.0, 0.7);
    adjust("Arm Angle",    1.00*MathUtil.applyDeadband(OI.joystick.getLeftX(),  0.1), -180.0, 0.0);
    double intake = OI.joystick.getRightTriggerAxis() - OI.joystick.getLeftTriggerAxis();
    adjust("Intake Angle",-1.00*MathUtil.applyDeadband(intake,                  0.1),    0.0, 120.0);

    if (OI.joystick.getAButtonPressed())
    {
      boolean extended = SmartDashboard.getBoolean("Arm Extended", false);
      SmartDashboard.putBoolean("Arm Extended", ! extended);
    }
  }
}
