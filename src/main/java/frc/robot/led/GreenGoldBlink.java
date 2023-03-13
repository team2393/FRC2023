// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GreenGoldBlink extends RepeatCommand
{
  public GreenGoldBlink(LED led)
  {
    super(new SequentialCommandGroup(new SetTwoColorsCommand(led, Color.kGold, Color.kGreen, 5),
                                     new WaitCommand(0.5),
                                     new SetTwoColorsCommand(led, Color.kGreen, Color.kGold, 5),
                                     new WaitCommand(0.5)
                                     ));
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}
