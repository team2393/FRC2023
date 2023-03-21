// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MovingColorList extends CommandBase
{
  private LED led;
  private Color[] colors;
  private int cycles;
  private int cycle = 0;

  /** @param led LED strip
   *  @param cycles Number of 50 Hz cycles to hold each step
   *  @param colors List of colors to use
   */
  public MovingColorList(LED led, int cycles, Color ... colors)
  {
    this.led = led;
    this.colors = colors;
    this.cycles = cycles;
    addRequirements(led);
  }

  @Override
  public void execute()
  {
    for (int i=0; i<LED.N; ++i)
      led.buffer.setLED(i, colors[(i+cycle/cycles) % colors.length]);
    led.set(led.buffer);
    ++cycle;
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}
