// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LookupTable;

public class FarCommand extends InteractiveArmLiftExtendCommand
{
  private static final LookupTable far_lookup = new LookupTable(
   new String[] { "Arm Angle", "Lift Height", "Extend" },
                          -90,           0.7,   0,
                          -70,           0.7,   0, 
                          -20,           0.7,   1);

  /** @param coordinator Coordinator */
  public FarCommand(Charm coordinator)
  {
    super(coordinator, far_lookup);
  }

  @Override
  public void initialize()
  {
    SmartDashboard.putString("Mode", "Far");
    // Preset arm, then allow interactive adjustment
    coordinator.arm_setpoint = -23.0;
  }
}