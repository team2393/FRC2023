// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.swervelib.SwerveDrivetrain;
import frc.robot.swervelib.SwerveModule;

/** Rotator using SparkMini for rotation, Falcon to drive, older pigeon as gyro */
public class MagnussenDriveTrain extends SwerveDrivetrain
{
  private final Pigeon2 gyro = new Pigeon2(0, RobotMap.CANIVORE);

  public MagnussenDriveTrain()
  {
    super(0.542,
          0.54,
          new SwerveModule[]
          {
            new SwerveModule(new FC_Rotator(0, -169.5+180), new FalconDriver(0)),
            new SwerveModule(new FC_Rotator(1,  -93.3+180), new FalconDriver(1)),
            new SwerveModule(new FC_Rotator(2,    8.9+180), new FalconDriver(2)),
            new SwerveModule(new FC_Rotator(3,   58.3+180), new FalconDriver(3))
          });
  }

  public double getRawHeading()
  {
    return gyro.getYaw();
  }

  public double getPitch()
  {
    if (RobotBase.isSimulation())
    {
      Pose2d pose = getPose();
      double x  = pose.getTranslation().getX();
      double heading = pose.getRotation().getDegrees();

      // Coming from the right, headed left, simulate driving 'up' and then 'down' the charge station
      if (Math.abs(Math.IEEEremainder(heading - 180.0, 360)) < 10.0)
      { // From the right, go 'up'
        if (4.03 < x  &&  x < 4.9)
          return 30.0;
        // Then flat from 4.03 to 3.66, then down
        if (2.8  < x  &&   x < 3.66)
          return -30.0;
      }
    }

    return -gyro.getPitch();
  }

  public double getRoll()
  {
    return -gyro.getRoll();
  }
}
