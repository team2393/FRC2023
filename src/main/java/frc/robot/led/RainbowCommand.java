// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RainbowCommand extends CommandBase
{
  private final LED led;
  private int phase = 0;

  public RainbowCommand(LED led)
  {
    this.led = led;
    addRequirements(led);
  }

  @Override
  public void execute()
  {
    for (int i=0; i<LED.N; ++i)
    {
      int hue = (phase + (i * 180 / LED.N)) % 180;
      led.buffer.setHSV(i, hue, 255, 255);
    }
    led.set(led.buffer);

    phase = (phase+1) % 180;
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}
