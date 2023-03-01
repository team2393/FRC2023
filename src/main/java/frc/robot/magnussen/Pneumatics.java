// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
  private Timer sim_timer = null;

  public Pneumatics()
  {
    // Enable the analog sensor, turn one below 85 psi, pump until 120 psi
    hub.enableCompressorAnalog(85.0, 120.0);
    nt_pressure = SmartDashboard.getEntry("Pressure");
  }
  
  @Override
  public void periodic()
  {
    if (RobotBase.isSimulation())
    { // Simulate air pressure.
      if (sim_timer == null)
      {
        nt_pressure.setDouble(0.0);
        // Start timer when enabled
        if (DriverStation.isEnabled())
        {
          sim_timer = new Timer();
          sim_timer.restart();
        }
      }
      else // Simulate pressure rising by 5 psi per second until 120
        nt_pressure.setDouble(Math.min(120.0, 5.0*sim_timer.get()));
    }
    else if (++calls > 50)
    {
      // Read analog input 0
      nt_pressure.setDouble(hub.getPressure(0));
      calls = 0;
    }
  }
}
