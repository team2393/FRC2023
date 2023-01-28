// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.demos.MotorTestRobot;
import frc.robot.magnussen.DriverTestRobot;
import frc.robot.magnussen.MagnussenRobot;
import frc.robot.magnussen.RotatorTestRobot;
import frc.robot.magnussen.SwerveModuleTestRobot;
import frc.robot.swervebot.SwerveBotRobot;

/** Java 'main' */
public final class Main
{ 
  /** Start one of the 'XXXRobot' robots */
  public static void main(String... args)
  {
    // CameraTestRobot
    RobotBase.startRobot(MagnussenRobot::new);
  }
}
