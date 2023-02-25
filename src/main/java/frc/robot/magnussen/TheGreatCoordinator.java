// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LookupTable;
import frc.robot.util.LookupTable.Entry;

/** The great arm/lift/grabber/intake coordinator */
public class TheGreatCoordinator extends SubsystemBase
{
  // Components that we handle
  private final Lift lift = new Lift();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();

  /** Setpoints
   *  If we adjusted based on the current value from
   *  getAngle(), getHeight(), ...,
   *  we would amplify any regulation error:
   *  Lift a little too low -> next time we make that the setpoint,
   *  moving it even lower.
   *  So we track and then adjust the _setpoint_.
   */
  private double lift_setpoint, arm_setpoint, intake_setpoint;

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
                                72,        -100,           0.3,
                                100,       -128,           0.2);

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
    reset();
  }

  public void reset()
  {
    lift_setpoint = lift.getHeight();
    arm_setpoint = arm.getAngle();
    intake_setpoint = intake.getAngle();
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("IntakeSP", intake_setpoint);
    SmartDashboard.putNumber("ArmSP", arm_setpoint);
    SmartDashboard.putNumber("LiftSP", lift_setpoint);
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
    SmartDashboard.putString("Mode", "STORE");
    mode = Mode.INTAKE;
    arm.extend(false);
    lift.setHeight(lift_setpoint = 0.0);
    lift.setHeight(lift_setpoint = 0.0);
    intake.setAngle(intake_setpoint = 125.0);
    arm.setAngle(arm_setpoint = -120);
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
    lift_setpoint   = adjust(lift_setpoint, -0.02*MathUtil.applyDeadband(OI.joystick.getRightY(),      0.1),    0.0, 0.7);
    arm_setpoint    = adjust(arm_setpoint,    1.00*MathUtil.applyDeadband(OI.joystick.getLeftX(),       0.1), -180.0, 0.0);
    intake_setpoint = adjust(intake_setpoint,-1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1),    0.0, 125.0);

    lift.setHeight (lift_setpoint);
    arm.setAngle   (arm_setpoint);
    intake.setAngle(intake_setpoint);

    if (OI.joystick.getAButtonPressed())
      arm.extend(! arm.isExtended());
  }

  private void handleIntake()
  {
    // Arm in
    arm.extend(false);
    
    // Move intake
    intake_setpoint = adjust(intake_setpoint, -1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), 0.0, 125.0);
    intake.setAngle(intake_setpoint);
    
    // Spinners turn on when intake is deployed?
    // TODO Or need another sensor?
    intake.setSpinner(intake.getAngle() < 90 ? Intake.SPINNER_VOLTAGE : 0);
    
    // Arm angle and lift follow intake
    Entry entry = intake_arm_lookup.lookup(intake_setpoint);
    arm.setAngle(arm_setpoint = entry.values[0]);
    lift.setHeight(lift_setpoint = entry.values[1]);

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
    intake.setAngle(intake_setpoint = 125.0);
    lift.setHeight(lift_setpoint = 0.3);
    intake.setSpinner(0);

    // Move arm angle
    arm_setpoint = adjust(arm_setpoint, 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -180.0, 0.0);
    arm.setAngle(arm_setpoint);

    // Extend when arm is sticking outside the back,
    // or out front yet not too far up (to prevent topping)
    // This should keep arm pulled in while where "inside" the robot
    arm.extend(arm_setpoint < -170  ||  (arm_setpoint > -80.0  &&  arm_setpoint < -40.0));

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
    intake.setAngle(intake_setpoint = 125.0);
    lift.setHeight(lift_setpoint = 0.3);
    arm.extend(false);
    intake.setSpinner(0);

    // Move arm angle
    arm_setpoint = adjust(arm_setpoint, 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -120.0, 0.0);
    arm.setAngle(arm_setpoint);

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
    intake.setAngle(intake_setpoint = 125.0);
    lift.setHeight(lift_setpoint = 0.7);
    arm.extend(lift.getHeight() > 0.4);
    intake.setSpinner(0);

    // Move arm angle, but not too far out front
    arm_setpoint = adjust(arm_setpoint, 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -120.0, -50.0);
    arm.setAngle(arm_setpoint);

    // Move to other mode?
    OI.selectIntakeNodeMode();
    OI.selectNearNodeMode();
    if (OI.selectMiddleNodeMode())
      mode = Mode.MID;
    OI.selectFarNodeMode();
  }
}
