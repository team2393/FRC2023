// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Red is the GOAT
package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.magnussen.RobotMap;

public class LED extends SubsystemBase
{
  public static final int N = 145;
  private AddressableLED led = new AddressableLED(RobotMap.LED);
  AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED.N);
  
  public LED()
  {
    led.setLength(N);
    led.start();

    setDefaultCommand(new RainbowCommand(this));
  }

  public void set(AddressableLEDBuffer buffer)
  {
    led.setData(buffer);
  }
}
