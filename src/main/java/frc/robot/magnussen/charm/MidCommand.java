// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LookupTable;

public class MidCommand extends InteractiveArmLiftExtendCommand
{
  private static final LookupTable mid_lookup = new LookupTable(
   new String[] { "Arm Angle", "Lift Height", "Extend" },
                          -90,           0.5,   0,
                          -62,           0.55,  0,   
                          -45,           0.56,  0,
                            0,           0.3,   0);

  /** @param coordinator Coordinator */
  public MidCommand(Charm coordinator)
  {
    super(coordinator, mid_lookup);
  }

  @Override
  public void initialize()
  {
    SmartDashboard.putString("Mode", "Mid");
    // Preset arm, then allow interactive adjustment
    coordinator.arm_setpoint = -50.0;
  }
}