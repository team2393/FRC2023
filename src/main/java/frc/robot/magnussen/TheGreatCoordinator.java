// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  final Grabber grabber = new Grabber();

  /** Setpoints
   *  If we adjusted based on the current value from
   *  getAngle(), getHeight(), ...,
   *  we would amplify any regulation error:
   *  Lift a little too low -> next time we make that the setpoint,
   *  moving it even lower.
   *  So we track and then adjust the _setpoint_.
   */
  private double lift_setpoint, arm_setpoint, intake_setpoint;

  /** Set lift to desired height or higher in case of conflict w/ arm or intake
   *  @param desired Desired height (minimum)
   */
  private void setSafeLiftHeight(double desired)
  {
    double lift_extra;

    // Arm far enough out, intake far enough in  => no conflict
    if (arm.getAngle() > -80  &&  intake.getAngle() > 100)
      lift_extra = 0.0;
    else
    {
      // How long is the arm right now?
      double arm_length = arm.isExtended() ? Arm.LENGTH_EXTENDED : Arm.LENGTH_RETRACTED;
      // How much of intake is sticking "up", how much of arm is "down"?
      double used_height = Intake.LENGTH * sin(toRadians(intake.getAngle()))
                         + arm_length    * sin(toRadians(-arm.getAngle()));
      // When lift is down, distance between arm and intake pivot points
      // is about 0.74 m, so need this extra hight to keep them apart:
      double extra_needed = used_height - 0.74;
      // Lift moves at about 30 deg to vertical, so needs to move a little more
      lift_extra = extra_needed/cos(toRadians(30));
    }

    // Use extra or desired, whatever's greater
    lift.setHeight(lift_setpoint = max(lift_extra, desired));
  }

  /** Base for command that uses the coordinator and shows its name as "Mode" */
  abstract private class CoordinatorCommand extends CommandBase
  {
    CoordinatorCommand()
    {
      addRequirements(TheGreatCoordinator.this);
    }

    @Override
    public void initialize()
    {
      String mode = getClass().getName();
      int sep = mode.indexOf('$');
      if (sep > 0)
        mode = mode.substring(sep+1);
      mode = mode.replace("Command", "");
      SmartDashboard.putString("Mode", mode);
    }
  }

  /** Directly control each device */
  private class DirectCommand extends CoordinatorCommand
  {
    @Override
    public void execute()
    {
      if (OI.joystick.getAButtonPressed())
        arm.extend(! arm.isExtended());
    
      setSafeLiftHeight(               adjust(lift_setpoint,  -0.02*MathUtil.applyDeadband(OI.joystick.getRightY(),      0.1),    0.0, 0.7));
      arm.setAngle   (arm_setpoint   = adjust(arm_setpoint,    1.00*MathUtil.applyDeadband(OI.joystick.getLeftX(),       0.1), -180.0, 0.0));
      intake.setAngle(intake_setpoint= adjust(intake_setpoint,-1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1),    0.0, 125.0));    
    }
  };

  /** Place everything in a safe position */
  private class StoreCommand extends CoordinatorCommand
  {
    @Override
    public void execute()
    {
      // Pull arm in, always safe to do
      arm.extend(false);
      // Pull intake and arm in
      intake.setAngle(intake_setpoint = 125.0);
      arm.setAngle(arm_setpoint = -100); // TODO Store arm at -90?
      setSafeLiftHeight(0.0);
      }
  };

  /** Take game piece in */
  private static final LookupTable cone_intake_arm_lookup = new LookupTable(
    new String[] { "Intake Angle", "Arm Angle", "Lift Height", "Extend" },
                                0,           0,           0.0,  0,
                               72,        -100,           0.3,  0,
                              100,        -128,           0.2,  0); 
  private static final LookupTable cube_intake_arm_lookup = new LookupTable(
    new String[] { "Intake Angle", "Arm Angle", "Lift Height", "Extend" },
                                0,         -80,          0.05,  1,
                               17,         -80,          0.4,   1,
                               30,        -100,          0.5,   0,
                               50,        -120,          0.5,   0);
  private class IntakeCommand extends CoordinatorCommand
  {
    @Override
    public void execute()
    {
      // Move intake
      intake_setpoint = adjust(intake_setpoint, -1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), 0.0, 125.0);
      intake.setAngle(intake_setpoint);
      
      // Spinners turn on when intake is deployed
      if (intake.getAngle() > 90)
        intake.setSpinner(0);
      else
        intake.setSpinner(OI.selectCubeIntake() ? Intake.SPINNER_VOLTAGE : -Intake.SPINNER_VOLTAGE);
      
      // Arm angle and lift follow intake
      Entry entry = OI.selectCubeIntake()
                  ? cube_intake_arm_lookup.lookup(intake_setpoint)
                  : cone_intake_arm_lookup.lookup(intake_setpoint);
      arm.setAngle(arm_setpoint = entry.values[0]);
      setSafeLiftHeight(entry.values[1]);
      arm.extend(entry.values[2] > 0.5);

      // Move to other mode?
      OI.selectIntakeMode();
      if (OI.selectNearNodeMode()  &&  intake_setpoint > 110)
        new NearCommand().schedule();
      if (OI.selectMiddleNodeMode()  &&  intake_setpoint > 110)
        new MidCommand().schedule();
      OI.selectFarNodeMode();
    }
  }

  private static final LookupTable near_lookup = new LookupTable(
    new String[] { "Arm Angle", "Lift Height", "Extend" },
                          -125,          0  ,         0,
                          -110,          0,           0,
                           -82,          0,           0,
                           -70,          0,           1,
                           -25,          0,           1,
                             0,          0,           1);
  private class NearCommand extends CoordinatorCommand
  {
    @Override
    public void execute()
    {
      // Intake in, lift at bottom
      intake.setAngle(intake_setpoint = 125.0);
      intake.setSpinner(0);
      
      // Move arm angle
      arm_setpoint = adjust(arm_setpoint, 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -125.0, 0.0);
      arm.setAngle(arm_setpoint);
      
      Entry entry = near_lookup.lookup(arm_setpoint);  
      setSafeLiftHeight(entry.values[0]);
      arm.extend(entry.values[1] > 0.5);

      // Move to other mode?
      if (OI.selectIntakeMode())
        startIntake();
      OI.selectNearNodeMode();
      if (OI.selectMiddleNodeMode())
        new MidCommand().schedule();
      OI.selectFarNodeMode();
    }
  }

  private final LookupTable mid_lookup = new LookupTable(
   new String[] { "Arm Angle", "Lift Height", "Extend" },
                          -90,           0.5,   0,
                          -65,           0.6,   0,   
                          -45,           0.56,  0,
                            0,           0.3,   0);
  private class MidCommand extends CoordinatorCommand
  {
    @Override
    public void execute()
    {
      // Intake in
      intake.setAngle(intake_setpoint = 125.0);
      intake.setSpinner(0);

      // Move arm angle
      arm_setpoint = adjust(arm_setpoint, 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -120.0, 0.0);
      arm.setAngle(arm_setpoint);

      Entry entry = mid_lookup.lookup(arm_setpoint);  
      setSafeLiftHeight(entry.values[0]);
      arm.extend(entry.values[1] > 0.5);

      // Move to other mode?
      if (OI.selectIntakeMode())
        startIntake();
      if (OI.selectNearNodeMode())
        new NearCommand().schedule();
      OI.selectMiddleNodeMode();
      if (OI.selectFarNodeMode())
        new FarCommand().schedule();
    }
  }

  private static final LookupTable far_lookup = new LookupTable(
   new String[] { "Arm Angle", "Lift Height", "Extend" },
                         -120,           0.7,   0,
                         -110,           0.7,   0,
                          -70,           0.7,   0, 
                          -25,           0.7,   1);
  private class FarCommand extends CoordinatorCommand
  {
    @Override
    public void execute()
    {
      // Intake in
      intake.setAngle(intake_setpoint = 125.0);
      intake.setSpinner(0);

      // Move arm angle
      arm_setpoint = adjust(arm_setpoint, 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -120.0, 0);
      arm.setAngle(arm_setpoint);

      Entry entry = far_lookup.lookup(arm_setpoint);  
      setSafeLiftHeight(entry.values[0]);
      arm.extend(entry.values[1] > 0.5);

      // Move to other mode?
      OI.selectIntakeMode();
      OI.selectNearNodeMode();
      if (OI.selectMiddleNodeMode())
        new MidCommand().schedule();
      OI.selectFarNodeMode();
    }
  }

  public TheGreatCoordinator()
  {
  }

  public void store()
  {
    new StoreCommand().schedule();
  }

  public void directControl()
  {
    new DirectCommand().schedule();
  }

  public void startIntake()
  {
    // TODO 2 Check intake sequence
    CommandBase grab_item = OI.selectCubeIntake() ? new GrabCubeCommand(grabber) : new GrabConeCommand(grabber);
    new SequentialCommandGroup(
      // Grab item and run intake, stop when we grabbed an item
      new ParallelDeadlineGroup(grab_item, new IntakeCommand()),
      // Next, move to 'mid' TODO better StoreCommand?
      new MidCommand()
    ).schedule();
  }

  @Override
  public void periodic()
  {
    if (DriverStation.isDisabled())
    {
      // Track current readings, so in case elements are manually moved,
      // we'll continue with those when again enabled
      lift_setpoint = lift.getHeight();
      arm_setpoint = arm.getAngle();
      intake_setpoint = intake.getAngle();
    }
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
}
