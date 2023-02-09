// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

/** Map of all inputs and outputs: CAN IDs etc. */
public class RobotMap
{
  /**
   * Power Distribution Module
   *
   * See
   * https://docs.wpilib.org/en/stable/docs/controls-overviews/control-system-hardware.html
   *
   * TODO Check actual fuse assignments
   * REV Power Distribution Hub
   * ==========================
   * 10 - 9 -
   * 11 - 8 -
   * 12 - 7 -
   * 13 - 6 -
   * 14 - 20 Amp back right encoder 5 - 20 Amp back left encoder
   * 15 - 40 Amp back right driver 4 - 40 Amp back left rotator
   * 16 - 40 Amp back right rotator 3 - 40 Amp back left driver
   * 17 - 20 Amp front right encoder 2 - 20 Amp front left encoder
   * 18 - 40 Amp front right driver 1 - 40 Amp front left rotator
   * 19 - 40 Amp front right rotator 0 - 40 Amp front left driver
   * 20 - 10 Amp RoboRIO
   * 21 - 10 Amp Radio power
   * 22 - 10 Amp Pigeon 2.0
   * 23 - 10 Amp Camera LED ring (switched)
   *
   * REV Mini Power Module
   * =====================
   * 0 - 3 Amp CanCoder
   * 1 - 3 Amp CanCoder
   * 2 - 3 Amp CanCoder
   * 3 - 3 Amp CanCoder
   * 4 -
   * 5 -
   * 
   * Remaining devices to be powered:
   *
   * 40 Amp Lift1
   * 40 Amp Lift2
   * 40 Amp Arm angle
   * 40 Amp Spinner
   *
   * 20 Amp Pneumatic hub -> Compressor
   * 
   * 10 Amp CANivore (optional?)
   * 5 Amp Pigeon (or plug into a nearby TalonFX?)
   * 10 Amp Limelight
   * 10 Amp Camera LED ring
   */

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
  public static final int LIFT1_ID = 13;
  public static final int LIFT2_ID = 14;

  /** DIO Arm angle sensor */
  public static final int ARM_ANGLE = 2;

  /** CAN ID for Arm motor */
  public static final int ARM_ID = 15;

  /** Pneumatic for Arm extender */
  public static final int ARM_EXTENDER = 0;

  /** CAN ID for Spinner */
  public static final int SPINNER_ID = 16;

  /** CAN ID for Intake motor */
  public static final int INTAKE_ID = -1;

  /** DIO Intake angle sensor */
  public static final int INTAKE_ANGLE = -1;
}
