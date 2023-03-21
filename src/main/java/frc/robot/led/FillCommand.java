// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FillCommand extends CommandBase
{
  private final LED led;
  private final Color color;
  private int filled, active;

  public FillCommand(LED led, Color color)
  {
    this.led = led;
    this.color = color;
    addRequirements(led);
  }

  @Override
  public void initialize()
  {
    filled = 0;
    active = 0;
  }

  @Override
  public void execute()
  {
    // active LED is white,
    // topmost 'filled' LEDs are colored

    // If LED.N is not even, that leaves the middle LED.
    // Blink red/blue
    led.buffer.setLED(LED.N/2,
                      (System.currentTimeMillis() / 500) % 2 == 1
                       ? Color.kFirstBlue : Color.kFirstRed);

    // Assume about half of the LED strip runs 'down',
    // then folds over to run 'up' other side of robot
    for (int i=0; i<LED.N/2; ++i)
    {
      if ((LED.N/2) - i == active)
      {
        led.buffer.setRGB(i, 255, 255, 255);
        led.buffer.setRGB(LED.N-1-i, 255, 255, 255);
      }
      else if (i <= filled)
      {
        led.buffer.setLED(i, color);
        led.buffer.setLED(LED.N-1-i, color);
      }
      else
      {
        led.buffer.setRGB(i, 0, 0, 0);
        led.buffer.setRGB(LED.N-1-i, 0, 0, 0);
      }
    }
    // Activate 'next' pixel.
    // Stepping by more than one makes it overall go faster.
    // Slow down with fill.
    active += Math.max(27-filled, 5);

    if (active >= LED.N/2 - filled)
    { // Active pixel reached the filled top
      active = 0;
      // Fill one more
      ++filled;
      if (filled >= LED.N/2)
      { // All filled? Start over
        active = 0;
        filled = 0;
      }
    }

    led.set(led.buffer);
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}
