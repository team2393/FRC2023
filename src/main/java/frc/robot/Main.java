// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.drivetrain.swerve.SwerveTestRobot;
import frc.robot.vision.CameraTestRobot;

/** Java 'main' */
public final class Main
{
  /** Start one of the 'XXXRobot' robots */
  public static void main(String... args)
  {
    // CameraTestRobot
    // SwerveTestRobot
    RobotBase.startRobot(CameraTestRobot::new);
  }
}
