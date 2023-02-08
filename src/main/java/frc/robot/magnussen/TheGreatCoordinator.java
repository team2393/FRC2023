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
    PRE_HOME_LIFT,
    /** Home lift */
    HOME_LIFT,
    /** Move arm etc. out of the way to provide intake with a clear path */
    CLEAR_INTAKE,
    /** Everything can move */
    NORMAL
  }

  private State state = State.PRE_HOME_LIFT;

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
    {
      // No 'else' here so we could go right away from
      // for example pre-home to home 
      if (state == State.PRE_HOME_LIFT)
        handlePreHome();
      if (state == State.HOME_LIFT)
        handleHome();
      if (state == State.CLEAR_INTAKE)
        handleClearIntake();
      if (state == State.NORMAL)
        handleNormal();
    }
  }

  private void handleDisabled()
  {
    // No motors are moving anyway, but just in case set all voltages to zero
    lift.setVoltage(0);
    arm.setVoltage(0);
    // TODO intake.setVoltage(0);

    // Once we re-enable, start over by homing the lift? 
    state = State.PRE_HOME_LIFT;
  }

  private void handlePreHome()
  {
    // Pre-home lift, move on when done
    if (lift.pre_home())
      state = State.HOME_LIFT;

    // Leave other systems wherever they are without voltage
    arm.setVoltage(0);
    // TODO intake.setVoltage(0);
  }

  private void handleHome()
  {
    // Home lift, move on when done
    if (lift.home())
      state = State.CLEAR_INTAKE;

    // Leave other systems wherever they are without voltage
    arm.setVoltage(0);
    // TODO intake.setVoltage(0);
  }

  private void handleClearIntake()
  {
      state = State.NORMAL;
/*
    // Is intake outside of the interference zone
    // and that's where it should be?
    if (intake_angle < 90  && intake.getAngle() < 90)
      state = State.NORMAL;
    else
    {
      // Intake is or should be in the interference zone.
      // Move everything else out of the way
      arm.setAngle(0);
      arm.extend(false);

      if (arm.getAngle() < -45)
      { // While arm is still getting out of the way, keep intake out
        intake.setAngle(0);
      }
      else // Allow intake to desired position
        intake.setAngle(intake_angle);
    }
*/
  }

  private void handleNormal()
  {
/*
    // Are we trying to move the intake into the interference zone?
    if (intake_angle > 90)
      state = State.CLEAR_INTAKE;
    else
    {
*/
      lift.setHeight(lift_height);
      arm.setAngle(arm_angle);
/*
      intake.setAngle(intake_angle);
    }
 */
  }
}
