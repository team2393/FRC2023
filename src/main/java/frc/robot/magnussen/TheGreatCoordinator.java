// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

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
  private final Lift lift = new Lift();
  private final Arm arm = new Arm();
  // private final Intake intake = new Intake();

  private boolean lift_homed = false;
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
    // If lift hasn't been homed, do that
    if (! lift_homed)
    {
      // TODO Before moving the lift down, do arm and intake
      // need to be in a certain position?
      lift_homed = lift.home();
    }
    else
    {
      // keep lift at desired position
      lift.setHeight(lift_height);
    }

    /*
    if (intake_angle > 90)
    {
      // To pull intake in, arm needs to be out of the way
      arm.setAngle(-100.0);
      if (arm.getAngle() < 90)
      {
        // Once arm is stored, we can move intake to desired location
        intake.setAngle(intake_angle);
      }
      else
      {
        // Keep intake just before 90 until arm has been stored
        intake.setAngle(85);
      }
    }
    else
    { // Intake is 'out', so arm can move as desired
      arm.setAngle(arm_angle);
      intake.setAngle(intake_angle);
    }
    */
  }
}
