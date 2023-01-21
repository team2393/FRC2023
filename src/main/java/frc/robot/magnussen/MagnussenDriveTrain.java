// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.swervelib.SwerveDrivetrain;
import frc.robot.swervelib.SwerveModule;

/** Rotator using SparkMini for rotation, Falcon to drive, older pigeon as gyro */
public class MagnussenDriveTrain extends SwerveDrivetrain
{
  private final Pigeon2 gyro = new Pigeon2(0, "rio");

  public MagnussenDriveTrain()
  {
    //TODO measure proper lengths
    super(0.6604,
          0.6604,
          new SwerveModule[]
          {
            //TODO configure offsets
            new SwerveModule(new FC_Rotator(0,  0), new FalconDriver(0)),
            new SwerveModule(new FC_Rotator(1,   0), new FalconDriver(1)),
            new SwerveModule(new FC_Rotator(2, 0), new FalconDriver(2)),
            new SwerveModule(new FC_Rotator(3, 0), new FalconDriver(3))
          });
  }

  public double getRawHeading()
  {
    return gyro.getYaw();
  }
}
