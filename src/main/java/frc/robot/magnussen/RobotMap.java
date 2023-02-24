// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

/** Map of all inputs and outputs: CAN IDs etc. */
public class RobotMap
{
  /** Name of CANivore */
  public static final String CANIVORE = "CANivore2393";

  // Following CAN devices are on CANIVORE
  // (but keep unique IDs so they can move)

  /** CAN IDs for driver motors */
  public static final int[] DRIVER_ID = new int[] { 1, 4, 10, 7 };

  /** CAN IDs for rotator motors */
  public static final int[] ROTATOR_ID = new int[] { 3, 2, 8, 9 };

  /** CAN IDs for rotator angle encoders */
  public static final int[] ANGLE_ID = new int[] { 5, 6, 12, 11 };

  // Remaining devices are on RIO

  /** DIO Lift bottom position sensor */
  public static final int LIFT_BOTTOM = 1;

  /** CAN ID for Lift motor */
  //skipped that number
  public static final int LIFT1_ID = 14;
  public static final int LIFT2_ID = 15;

  /** CAN ID for Arm motor (also connected to arm angle encoder) */
  public static final int ARM_ID = 16;

  /** Pneumatic for Arm extender */
  public static final int ARM_EXTENDER = 0;

  /** CAN ID for Spinner on grabber */
  public static final int GRABBER_ID = 17;

  /** DIO port for cube/cone sensor */
  public static final int GRABBER_SENSOR = 9;

  /** CAN ID for Intake motor (also used for intake angle encoder) */
  public static final int INTAKE_ID = 18;

  /** CAN ID for Intake spinner  */
  public static final int INTAKE_SPINNER = 19;

  /** DIO for rev throughbore encoder */
  public static final int INTAKE_ANGLE = 4;

}
