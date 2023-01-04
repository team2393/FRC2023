// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Java 'main' */
public final class Main
{
    /** Start one of the 'XXXRobot' robots */
    public static void main(String... args)
    {
        RobotBase.startRobot(TestRobot::new);
    }
}