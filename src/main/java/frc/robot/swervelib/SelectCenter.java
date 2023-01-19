// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swervelib;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Selects the 'center' of rotation
 *
 *  Starts out and can be reset to (0, 0),
 *  and move to the 4 corners of the robot or beyond.
 */
public class SelectCenter
{
  private final double x_step, y_step;  
  private Translation2d center = new Translation2d(0, 0);

  public SelectCenter(SwerveDrivetrain drivetrain)
  {
    x_step = drivetrain.getLength()/2;
    y_step = drivetrain.getWidth()/2;
  }

  public Translation2d determineCenter()
  {
    if (SwerveOI.resetCenter())
        center = new Translation2d(0, 0);
    if (SwerveOI.leftCenter())
        center = new Translation2d(center.getX(),          center.getY() + y_step);
    if (SwerveOI.rightCenter())
        center = new Translation2d(center.getX(),          center.getY() - y_step);
    if (SwerveOI.frontCenter())
        center = new Translation2d(center.getX() + x_step, center.getY());
    if (SwerveOI.backCenter())
        center = new Translation2d(center.getX() - x_step, center.getY());
    SmartDashboard.putNumber("CenterX", center.getX());
    SmartDashboard.putNumber("CenterY", center.getY());
    return center;
  }
}


