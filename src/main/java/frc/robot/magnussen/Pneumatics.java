// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Pneumatics
 *
 *  The analog output of the sensor must be connected to analog input 0 of the hub
 */
public class Pneumatics extends SubsystemBase
{
  private final PneumaticHub hub = new PneumaticHub();
  private final NetworkTableEntry nt_pressure;
  private int calls = 0;

  public Pneumatics()
  {
    // Enable the analog sensor, turn one below 85 psi, pump until 120 psi
    hub.enableCompressorAnalog(85.0, 120.0);
    nt_pressure = SmartDashboard.getEntry("Pressure");
  }
  
  @Override
  public void periodic()
  {
    if (++calls > 50)
    {
      // Read analog input 0
      nt_pressure.setDouble(hub.getPressure(0));
      calls = 0;
    }
  }
}
