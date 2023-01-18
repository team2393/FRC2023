// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public void resetPosition()
  {
    driver.resetPosition();
  }

  public Rotation2d getAngle()
  {
    return rotator.getAngle();
  }

  public SwerveModulePosition getPosition()
  {
    return new SwerveModulePosition(driver.getPosition(), rotator.getAngle());
  }

  public void drive(double angle, double speed)
  {
     rotator.setAngle(angle);
     driver.setSpeed(speed);
  }
}
