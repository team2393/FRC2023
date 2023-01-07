// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Selects the 'center' of rotation
 *
 *  Starts out and can be reset to (0, 0),
 *  and move to the 4 corners of the robot or beyond.
 */
public class SelectCenter
{
  private Translation2d center = new Translation2d(0, 0);

  public Translation2d determineCenter()
  {
    if (OI.resetCenter())
        center = new Translation2d(0, 0);
    if (OI.leftCenter())
        center = new Translation2d(center.getX(), center.getY() + Drivetrain.WIDTH / 2);
    if (OI.rightCenter())
        center = new Translation2d(center.getX(), center.getY() - Drivetrain.WIDTH / 2);
    if (OI.frontCenter())
        center = new Translation2d(center.getX() + Drivetrain.LENGTH / 2, center.getY());
    if (OI.backCenter())
        center = new Translation2d(center.getX()- Drivetrain.LENGTH / 2, center.getY());
    SmartDashboard.putNumber("CenterX", center.getX());
    SmartDashboard.putNumber("CenterY", center.getY());
    return center;
  }
}


