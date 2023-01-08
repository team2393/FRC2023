// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import frc.robot.CommandBaseRobot;

public class CameraTestRobot extends CommandBaseRobot
{
  @Override
  public void robotInit()
  {
    super.robotInit();
  
    final int width = 320, height = 240;
    final CameraThread thread = new CameraThread(width, height, () -> new AprilTagPipeline(width, height));
    thread.start();
  }
}
