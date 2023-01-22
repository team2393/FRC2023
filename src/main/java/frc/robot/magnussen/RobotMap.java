// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

/** Map of all inputs and outputs: CAN IDs etc. */
public class RobotMap
{
  /** Power Distribution Module        TODO Check actual fuse assignments
   * 
   *  See https://docs.wpilib.org/en/stable/docs/controls-overviews/control-system-hardware.html
   * 
   *  REV Power Distribution Hub
   *  10 - 20 Amp front left rotator           9 - 40 Amp front left driver
   *  11 -                                     8 - 40 Amp front right driver
   *  12 -                                     7 - 
   *  13 -                                     6 - 
   *  14 -                                     5 - 
   *  15 -                                     4 - 
   *  16 -                                     3 - 
   *  17 -                                     2 - 
   *  18 -                                     1 - 
   *  19 - 20 Amp Pneumatic hub -> Compressor  0 - 
   *  20 - 10 Amp Radio power
   *  21 - 10 Amp RoboRIO
   *  22 - 
   *  23 - (switched)
   * 
   *  40 Amp Front left driver
   *  40 Amp Front right driver
   *  40 Amp Back right driver
   *  40 Amp Back left driver
   *  40 Amp Lift
   *  40 Amp Arm angle
   *  40 Amp Spinner1
   *  40 Amp Spinner2
   *
   *  20 Amp Front left rotator
   *  20 Amp Front right rotator
   *  20 Amp Back right rotator
   *  20 Amp Back left rotator
   *  20 Amp Pneumatic hub -> Compressor
   * 
   *  10 Amp CANivore (optional?)
   *  10 Amp Front left CANcoder
   *  10 Amp Front right CANcoder
   *  10 Amp Back right CANcoder
   *  10 Amp Back left CANcoder
   *  10 Amp Pigeon
   *  10 Amp Limelight
   *  10 Amp Camera LED ring
   */

  /** Name of CANivore */
  public static final String CANIVORE = "CANivore2393";

  // Following CAN devices are on CANIVORE
  // (but keep unique IDs so they can move)

  /** CAN IDs for driver motors */
  public static final int[] DRIVER_ID = new int[] { 1, 2, 3, 4 };

  /** CAN IDs for rotator motors */
  public static final int[] ROTATOR_ID = new int[] { 5, 6, 7, 8 };

  /** CAN IDs for rotator angle encoders */
  public static final int[] ANGLE_ID = new int[] { 1, 2, 3, 4 };

  // Remaining devices are on RIO

  /** DIO Lift bottom position sensor */
  public static final int LIFT_BOTTOM = 1;

  /** CAN ID for Lift motor */
  public static final int LIFT_ID = 9;

  /** DIO Arm angle sensor */
  public static final int ARM_ANGLE = 2;

  /** CAN ID for Arm motor */
  public static final int ARM_ID = 10;

  /** Pneumatic for Arm extender */
  public static final int ARM_EXTENDER = 0;

  /** CAN IDs for Spinner */
  public static final int SPINNER1_ID = 11;
  public static final int SPINNER2_ID = 12;
}
