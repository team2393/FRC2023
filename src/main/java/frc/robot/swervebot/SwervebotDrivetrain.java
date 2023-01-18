// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervebot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.swervelib.SwerveDrivetrain;
import frc.robot.swervelib.SwerveModule;

/** Rotator using SparkMini and absolute encoder */
public class SwervebotDrivetrain extends SwerveDrivetrain
{
  private final PigeonIMU pigeon = new PigeonIMU(0);

  public SwervebotDrivetrain()
  {
    super(0.64135,
          0.61595,
          new SwerveModule[]
          {
            new SwerveModule(new SparkMiniRotator(0,  -18), new FalconDriver(0)),
            new SwerveModule(new SparkMiniRotator(1,   90), new FalconDriver(1)),
            new SwerveModule(new SparkMiniRotator(2, -161), new FalconDriver(2)),
            new SwerveModule(new SparkMiniRotator(3, -104), new FalconDriver(3))
          });
  }

  public double getRawHeading()
  {
    return pigeon.getFusedHeading();
  }
}
