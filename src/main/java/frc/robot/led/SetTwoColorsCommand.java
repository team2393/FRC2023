// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetTwoColorsCommand extends InstantCommand
{
  /** @param led LED
   *  @param one First color
   *  @param other Other color
   *  @param pixels Number of pixel between color changes
   */
  public SetTwoColorsCommand(LED led, Color one, Color other, int pixels)
  {
    super(() ->
    {
      for (int i=0; i<LED.N; ++i)
        if ((i / pixels) % 2 == 0)
          led.buffer.setLED(i, one);
        else
          led.buffer.setLED(i, other);
      led.set(led.buffer);
      }, led);
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}
