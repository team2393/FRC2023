// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetAllLEDsCommand extends CommandBase
{
  private final LED led;
  private final int r, g, b;

  public SetAllLEDsCommand(LED led, int r, int g, int b)
  {
    this.led = led;
    this.r = r;
    this.g = g;
    this.b = b;
    addRequirements(led);
  }

  @Override
  public void initialize()
  {
    for (int i=0; i<LED.N; ++i)
      led.buffer.setRGB(i, r, g, b);
    led.set(led.buffer);
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }
}
