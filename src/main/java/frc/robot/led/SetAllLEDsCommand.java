// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetAllLEDsCommand extends InstantCommand
{
  /** @param led LED
   *  @param color Color
   */
  public SetAllLEDsCommand(LED led, Color color)
  {
    super(() ->
    {
      for (int i=0; i<LED.N; ++i)
        led.buffer.setLED(i, color);
      led.set(led.buffer);
    }, led);
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}
