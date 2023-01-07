// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroHelper extends SubsystemBase
{
  private final PigeonIMU pigeon = new PigeonIMU(0);
  private final NetworkTableEntry nt_gyro = SmartDashboard.getEntry("Gyro");
  private double zero_offset = 0.0;

  public GyroHelper()
  {
    // Seemed logical but result in "CTRE: Feature Not Supported"
    // pigeon.configFactoryDefault();
    // pigeon.clearStickyFaults();

    // Publish command for resetting the gyro
    // CommandBase reset = new CommandBase()
    // {
    //   @Override
    //   public void execute()
    //   {
    //     reset();
    //   }
    //
    //   @Override
    //   public boolean isFinished()
    //   {
    //     return true;
    //   }
    //
    //   @Override
    //   public boolean runsWhenDisabled()
    //   {
    //     return true;
    //   }
    // };
    CommandBase reset = Commands.runOnce(this::reset)
                                .ignoringDisable(true);
    reset.setName("ResetGyro");
    SmartDashboard.putData(reset);
  }

  /** Reset heading to zero degrees */
  public void reset()
  {
    // Could try this via
    //   pigeon.setFusedHeading(0.0);
    // but that merely sends a "set heading to zero" message
    // via the CAN bus, which takes a little time.
    // On the next reading, we don't know if the pigeon
    // already received that message or not,
    // so we can get non-zero readings for long enough
    // to mess up any autonomous code.
    // Suggested solution is to keep our own zero offset
    zero_offset = pigeon.getFusedHeading();
  }

  /** @return Heading in degrees */
  private double getAngle()
  {
    return pigeon.getFusedHeading() - zero_offset;
  }

  @Override
  public void periodic()
  {
    nt_gyro.setDouble(getAngle());
  }
}
