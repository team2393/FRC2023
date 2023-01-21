// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

/** Map of all inputs and outputs: CAN IDs etc. */
public class RobotMap
{
  /** Power Distribution Module
   * 
   *  TODO Check actual fuse assignments
   * 
   *  40 Amp
   *  Front left driver falcon
   *  Front right driver falcon
   *  Back right driver falcon
   *  Back left driver falcon
   *
   *  20 Amp
   *  Front left rotator falcon
   *  Front right rotator falcon
   *  Back right rotator falcon
   *  Back left rotator falcon
   * 
   *  10 Amp
   *  Front left encoder CANcoder
   *  Front right rotator CANcoder
   *  Back right rotator CANcoder
   *  Back left rotator CANcoder
   *  Pigeon
   * 
   *  ?? Amp
   *  Lift
   *  Arm angle
   *  Spinner1
   *  Spinner2
   *  Compressor
   */

  /** Name of CANivore */
  public static final String CANIVORE = "CANivore2393";

  // Following CAN devices are on CANIVORE

  /** CAN IDs for driver motors */
  public static final int[] DRIVER_ID = new int[] { 1, 2, 3, 4 };

  /** CAN IDs for rotator motors */
  public static final int[] ROTATOR_ID = new int[] { 5, 6, 7, 8 };

  /** CAN IDs for rotator angle encoders */
  public static final int[] ANGLE_ID = new int[] { 1, 2, 3, 4 };

  // Remaining CAN devices are on RIO

  /** DIO Lift bottom position sensor */
  public static final int LIFT_HOME = 1;

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
