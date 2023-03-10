// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.magnussen.OI;
import frc.robot.util.LookupTable;
import frc.robot.util.LookupTable.Entry;

public class InteractiveArmLiftExtendCommand extends CommandBase
{
  protected final Charm coordinator;
  protected final LookupTable table;

  /** @param coordinator Coordinator
   *  @param table Table to use for arm angle, lift height, arm extension
   */
  public InteractiveArmLiftExtendCommand(Charm coordinator, LookupTable table)
  {
    this.coordinator = coordinator;
    this.table = table;
    addRequirements(coordinator);
  }
  
  protected double getUserInput()
  {
    return MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1);
  }

  @Override
  public void execute()
  {
    // Lookup settings for adjusted arm angle
    Entry entry = table.lookup(coordinator.arm_setpoint + getUserInput());
    // Use resulting values for arm(!), lift, extension
    coordinator.arm_setpoint = entry.position;
    coordinator.lift_setpoint = entry.values[0];
    coordinator.arm.extend(entry.values[1] > 0.5);
  }
}