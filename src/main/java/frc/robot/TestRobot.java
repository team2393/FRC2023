// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Test robot
 *  Shows state of 'USER' button on dashboard
 */
public class TestRobot extends TimedRobot
{
  private Command show_user_button = new CommandBase()
  {
    @Override
    public void execute()
    {
      SmartDashboard.putBoolean("UserButton", RobotController.getUserButton());
    }
  };

  @Override
  public void robotInit()
  {
    System.out.println("************************************");
    System.out.println("**  " + getClass().getName());
    System.out.println("************************************");
  }

  @Override
  public void robotPeriodic()
  {
    // Support commmand framework
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit()
  {
    show_user_button.schedule();
  }
}
