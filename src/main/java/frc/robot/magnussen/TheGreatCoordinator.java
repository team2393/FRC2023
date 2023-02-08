// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The great arm/lift/grabber/intake coordinator
 * 
 *  At startup, the lift needs to be 'homed' via the bottom switch.
 * 
 *  When moving the various components, there are some constraints.
 *  For example, the intake can only move all the way "in"
 *  when the arm is inside the lift.
 * 
 *  This helper allows setting the desired lift height etc
 *  and then tries to get there.
 */
public class TheGreatCoordinator extends SubsystemBase
{
  // Devices that we handle
  private final Lift lift = new Lift();
  private final Arm arm = new Arm();
  // private final Intake intake = new Intake();

  enum State
  {
    /** Prepare to home lift by moving it slightly 'up' off the switch */
    PREP_LIFT,
    /** Home lift */
    HOME_LIFT,
    /** Move arm etc. out of the way to provide intake with a clear path */
    CLEAR_INTAKE,
    /** Everything can move */
    NORMAL
  }

  private State state = State.PREP_LIFT;

  private double lift_height = 0.1;
  private double arm_angle = 0.0;
  private double intake_angle = 0.0;

  /** @param meters Desired lift height */
  public void setLiftHeight(double meters)
  {
    lift_height = meters;
  }

  /** @param degrees Desired arm angle */
  public void setArmAngle(double degrees)
  {
    arm_angle = degrees;
  }

  /** @param degrees Desired intake angle */
  public void setIntakeAngle(double degrees)
  {
    intake_angle = degrees;
  }
  
  @Override
  public void periodic()
  {
    if (RobotState.isDisabled())
      handleDisabled();
    else
      handleEnabled();
  }

  private void handleDisabled()
  {
    // No motors are moving.
    // Once we re-enable, start over by homing the lift? 
    state = State.PREP_LIFT;
  }

  private void handleEnabled()
  {
    if (state == State.PREP_LIFT)
    {

    }
    if (state == State.HOME_LIFT)
    {
      arm.setAngle(-90);
      // intake.setAngle(45);
      if (lift.home())
        state = State.NORMAL;
    }
    if (state == State.NORMAL)
    {

    }
    if (state == State.CLEAR_INTAKE)
    {

    }
  }
}
