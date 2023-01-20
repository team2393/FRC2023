package frc.robot.swervelib;
// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** One serve module that can rotate and drive */
public class SwerveModule extends SubsystemBase
{
  private final Rotator rotator;
  private final Driver driver;    

  public SwerveModule(Rotator rotator,
                      Driver driver)
  {
    this.rotator = rotator;
    this.driver = driver;
  }

  /** Reset position of driver to zero */
  public void resetPosition()
  {
    driver.resetPosition();
  }

  /** @return Angle of rotator */
  public Rotation2d getAngle()
  {
    return rotator.getAngle();
  }

  /** @return Driver position */
  public SwerveModulePosition getPosition()
  {
    return new SwerveModulePosition(driver.getPosition(), rotator.getAngle());
  }

  /** @param angle Module angle in degrees
   *  @param speed Module speed in meters per second
   */
  public void drive(double angle, double speed)
  {
     rotator.setAngle(angle);
     driver.setSpeed(speed);
  }
}