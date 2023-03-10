// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Java 'main' */
public final class Main
{ 
  /** Start one of the 'XXXRobot' robots */
  public static void main(String... args)
  {
    RobotBase.startRobot(frc.robot.magnussen.MagnussenRobot::new);
    //  RobotBase.startRobot(frc.robot.magnussen.CharmTestRobot::new);
    // RobotBase.startRobot(frc.robot.magnussen.SecondAttemptTestRobot::new);
    // RobotBase.startRobot(frc.robot.magnussen.TheGreatCoordinatorTestRobot::new);
    // RobotBase.startRobot(frc.robot.magnussen.IntakeTestRobot::new);
    // RobotBase.startRobot(frc.robot.magnussen.ArmTestRobot::new);
    // RobotBase.startRobot(frc.robot.magnussen.GrabberTestRobot::new);
    // RobotBase.startRobot(frc.robot.swervebot.SwerveBotRobot::new);
    // RobotBase.startRobot(frc.robot.magnussen.LiftTestRobot::new);
  }
}
