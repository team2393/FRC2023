package frc.robot.drivetrain.vision;

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.CommandBaseRobot;

public class CameraTestRobot extends CommandBaseRobot
{
  @Override
  public void robotInit()
  {
    super.robotInit();
    CameraServer.startAutomaticCapture();
  }
}
