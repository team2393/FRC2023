// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LookupTable;
import frc.robot.util.LookupTable.Entry;

/** The great arm/lift/grabber/intake coordinator */
public class TheGreatCoordinator
{
  // Components that we handle
  private final Lift lift = new Lift();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();

  /** Modes:
   *  DIRECT is for initial testing.
   *  Driver has to directly control the lift, arm and intake,
   *  no smarts at all
   * 
   *  Remaining modes control all components
   *  based on a single joystick input
   */
  public enum Mode
  {
    /** Directly control each component */
    DIRECT,
    /** Move intake */
    INTAKE,
    /** Position grabber for a "near" node */
    NEAR,
    /** Position grabber for a "mid-distance" node */
    MID,
    /** Position grabber for a "far" node */
    FAR
  }

  private Mode mode;

  /** Lookup from intake angle to arm angle and lift height */
  private static final LookupTable intake_arm_lookup = new LookupTable(
    new String[] { "Intake Angle", "Arm Angle", "Lift Height" },
                                0,           0,            0.0,
                               45,         -20,            0.0,
                               90,         -90,            0.3,
                              100,        -120,            0.0);

  // Demo of intake_arm_lookup
  public static void main(String[] args)
  {
    for (double intake_angle=0;  intake_angle < 150; intake_angle += 5)
    {
      double arm_angle = intake_arm_lookup.lookup(intake_angle).getValue();
      System.out.format("Intake %5.1f deg  ->  arm %5.1f deg\n", intake_angle, arm_angle);
    }
  }


  /** @param use_modes Use modes? */
  public TheGreatCoordinator(boolean use_modes)
  {
    mode = use_modes ? Mode.INTAKE : Mode.DIRECT;
  }
  
  /** @param value Current value
   *  @param rate Decrease/increase -1..1
   *  @param min Minimum value
   *  @param max Maximum value
   *  @return Adjusted value
   */
  private double adjust(double value, double rate, double min, double max)
  {
    value += rate;
    return MathUtil.clamp(value, min, max);
  }

  /** Place everything in a save position */
  public void store ()
  {
    mode = Mode.INTAKE;
    arm.extend(false);
    lift.setHeight(0.0);
    intake.setAngle(120.0);
    arm.setAngle(-100);
  }

  /** Interactively run the intake, arm, lift, grabber */
  public void run()
  {
    SmartDashboard.putString("Mode", mode.name());

    // Really same as
    //   if (mode == Mode.INTAKE)
    //     handleIntake();
    //   else if (mode == Mode.NEAR)
    // except switch (..) will show a compiler warning
    // if we forget to handle one of the options
    switch (mode)
    {
    case DIRECT: directControl(); break;
    case INTAKE: handleIntake();  break;
    case NEAR:   handleNear();    break;
    case MID:    handleMid();     break;
    case FAR:    handleFar();     break;
    }
  }

  /** Directly control each device */
  private void directControl()
  {
    lift.setHeight (adjust(lift.getHeight(), -0.02*MathUtil.applyDeadband(OI.joystick.getRightY(),      0.1),    0.0, 0.7));
    arm.setAngle   (adjust(arm.getAngle(),    1.00*MathUtil.applyDeadband(OI.joystick.getLeftX(),       0.1), -180.0, 0.0));
    intake.setAngle(adjust(intake.getAngle(),-1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1),    0.0, 120.0));

    if (OI.joystick.getAButtonPressed())
      arm.extend(! arm.isExtended());
  }

  private void handleIntake()
  {
    // Arm in
    arm.extend(false);
    
    // Move intake
    double intake_angle = adjust(intake.getAngle(), -1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), 0.0, 120.0);
    intake.setAngle(intake_angle);
    
    // Spinners turn on when intake is deployed?
    // TODO Or need another sensor?
    intake.setSpinner(intake.getAngle() < 90 ? Intake.SPINNER_VOLTAGE : 0);
    
    // Arm angle and lift follow intake
    Entry entry = intake_arm_lookup.lookup(intake_angle);
    arm.setAngle(entry.values[0]);
    lift.setHeight(entry.values[1]);

    // Move to other mode?
    OI.selectIntakeNodeMode();
    if (OI.selectNearNodeMode())
      mode = Mode.NEAR;
    if (OI.selectMiddleNodeMode())
      mode = Mode.MID;
    OI.selectFarNodeMode();
  }

  private void handleNear()
  {
    // Intake in, lift at bottom
    intake.setAngle(120.0);
    lift.setHeight(0.0);
    intake.setSpinner(0);

    // Move arm angle
    double arm_angle = adjust(arm.getAngle(), 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -180.0, 0.0);
    arm.setAngle(arm_angle);

    // Extend when arm is sticking outside the back,
    // or out front yet not too far up (to prevent topping)
    // This should keep arm pulled in while where "inside" the robot
    arm.extend(arm_angle < -170  ||  (arm_angle > -80.0  &&  arm_angle < -40.0));

    // Move to other mode?
    if (OI.selectIntakeNodeMode())
      mode = Mode.INTAKE;
    OI.selectNearNodeMode();
    if (OI.selectMiddleNodeMode())
      mode = Mode.MID;
    OI.selectFarNodeMode();
  }

  private void handleMid()
  {
    // Intake in, lift at mid, arm in
    intake.setAngle(120.0);
    lift.setHeight(0.3);
    arm.extend(false);
    intake.setSpinner(0);

    // Move arm angle
    arm.setAngle(adjust(arm.getAngle(), 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -120.0, 0.0));

    // Move to other mode?
    if (OI.selectIntakeNodeMode())
      mode = Mode.INTAKE;
    if (OI.selectNearNodeMode())
      mode = Mode.NEAR;
    OI.selectMiddleNodeMode();
    if (OI.selectFarNodeMode())
      mode = Mode.FAR;
  }

  private void handleFar()
  {
    // Intake in, lift all up, arm out as soon as lift high enough
    intake.setAngle(120.0);
    lift.setHeight(0.7);
    arm.extend(lift.getHeight() > 0.4);
    intake.setSpinner(0);

    // Move arm angle, but not too far out front
    arm.setAngle(adjust(arm.getAngle(), 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -120.0, -50.0));

    // Move to other mode?
    OI.selectIntakeNodeMode();
    OI.selectNearNodeMode();
    if (OI.selectMiddleNodeMode())
      mode = Mode.MID;
    OI.selectFarNodeMode();
  }
}
