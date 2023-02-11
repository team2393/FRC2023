// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The great arm/lift/grabber/intake coordinator
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
    /** Move arm etc. out of the way to provide intake with a clear path */
    CLEAR_INTAKE,
    /** Everything can move */
    NORMAL
  }

  private State state = State.NORMAL;

  private double lift_height = 0.1;
  private double arm_angle = 0.0;
  private boolean arm_extended = false;
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

  /** @param extend Extend arm? */
  public void setArmExtension(boolean extend)
  {
    arm_extended = extend;
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
      if (state == State.NORMAL)
        handleNormal();
      if (state == State.CLEAR_INTAKE)
        handleClearIntake();
    }
  }

  private void handleDisabled()
  {
    // No motors are moving anyway, but just in case set all voltages to zero
    lift.setVoltage(0);
    arm.setVoltage(0);
    arm.extend(false);
    // TODO intake.setVoltage(0);

    // Once we re-enable, start over in NORMAL?
    state = State.NORMAL;
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
      arm.extend(arm_extended);
/*
      intake.setAngle(intake_angle);
    }
 */
  }

  private void handleClearIntake()
  {
/*
    // Is intake outside of the interference zone
    // and that's where it should be?
    if (intake.getAngle() < 90  &&  intake_angle < 90)
    {
      // All can move as desired ...
      lift.setHeight(lift_height);
      arm.setAngle(arm_angle);
      arm.extend(arm_extended);
      intake.setAngle(intake_angle);
      // .. and next period we'll be in normal state
      state = State.NORMAL;
    }
    else
    {
      // Intake is or should be in the interference zone.
      // Move everything else out of the way
      arm.setAngle(0);
      arm.extend(false);
      // Lift can move as desired?
      lift.setHeight(lift_height);

      if (arm.getAngle() < -45)
      { // While arm is still getting out of the way, keep intake out
        intake.setAngle(0);
      }
      else // Allow intake to desired position
        intake.setAngle(intake_angle);
    }
*/
  }
}
