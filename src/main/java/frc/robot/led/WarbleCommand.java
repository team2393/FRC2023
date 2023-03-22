// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Warble one color.
 *  Impossible to describe, needs to be experienced.
 */
public class WarbleCommand extends CommandBase
{
  private final LED led;
  private final Color color;

  public WarbleCommand(LED led, Color color)
  {
    this.led = led;
    this.color = color;
    addRequirements(led);
  }

  @Override
  public void execute()
  {
    // Use sine to scale intensity (black to full) of the color by 0...1 every 2000ms (2 sec)
    double factor = (1.0 + Math.sin(2*Math.PI*System.currentTimeMillis()/2000)) / 2.0;
    Color variant = new Color(factor*color.red, factor*color.green, factor*color.blue);
    // Set all LEDs to that color
    for (int i=0; i<LED.N; ++i)
      led.buffer.setLED(i, variant);
    // Set 10 random pixels to white, black, and the full color
    for (int i=0; i<10; ++i)
    {
      // Math.random is [0...1[, always less than 1, so we should get 0...(LED.N-1)
      led.buffer.setLED((int)(Math.random()*LED.N), Color.kWhiteSmoke);
      led.buffer.setLED((int)(Math.random()*LED.N), Color.kBlack);
      led.buffer.setLED((int)(Math.random()*LED.N), color);
    }

    led.set(led.buffer);
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}
