// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LookupTable;

public class NearCommand extends InteractiveArmLiftExtendCommand
{
  private static final LookupTable near_lookup = new LookupTable(
    new String[] { "Arm Angle", "Lift Height", "Extend" },
                           -90,          0,           0,
                           -82,          0,           0,
                           -70,          0,           1,
                             0,          0,           1);

  /** @param coordinator Coordinator */
  public NearCommand(Charm coordinator)
  {
    super(coordinator, near_lookup);
  }

  @Override
  public void initialize()
  {
    SmartDashboard.putString("Mode", "Near");
    // Preset arm, then allow interactive adjustment
    coordinator.arm_setpoint = -70.0;
  }
}