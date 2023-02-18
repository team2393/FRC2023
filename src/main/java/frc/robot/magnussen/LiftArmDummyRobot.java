// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Fake lift/arm/... Robot
 *
 *  Used to simulate a robot where we control the lift/arm/intake
 */
public class LiftArmDummyRobot extends CommandBaseRobot
{
  private final Lift lift = new Lift();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();

  /** @param getter How to read the current value
   *  @param setter How to set the desired value
   *  @param rate Decrease/increase -1..1
   *  @param min Minimum value
   *  @param max Maximum value
   *  @return Value that we set
   */
  private double adjust(DoubleSupplier getter, DoubleConsumer setter, double rate, double min, double max)
  {
    double value = getter.getAsDouble();
    value += rate;
    setter.accept(MathUtil.clamp(value, min, max));
    return value;
  }

  @Override
  public void teleopPeriodic()
  {
    // directControl();
    handleModes();
  }

  /** Directly control each device */
  private void directControl()
  {
    adjust(lift::getHeight, lift::setHeight,  -0.02*MathUtil.applyDeadband(OI.joystick.getRightY(),      0.1),    0.0, 0.7);
    adjust(arm::getAngle,   arm::setAngle,     1.00*MathUtil.applyDeadband(OI.joystick.getLeftX(),       0.1), -180.0, 0.0);
    adjust(intake::getAngle, intake::setAngle,-1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1),    0.0, 120.0);

    if (OI.joystick.getAButtonPressed())
      arm.extend(! arm.isExtended());
  }

  private enum Mode
  {
    /** Move intake */
    INTAKE,
    /** Position grabber for a "near" node */
    NEAR,
    /** Position grabber for a "mid-distance" node */
    MID,
    /** Position grabber for a "far" node */
    FAR
  }

  private Mode mode = Mode.INTAKE;

  private void handleModes()
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
    case INTAKE: handleIntake(); break;
    case NEAR:   handleNear();   break;
    case MID:    handleMid();    break;
    case FAR:    handleFar();    break;
    }
  }

  private void handleIntake()
  {
    // Fix: Arm in, lift at bottom
    arm.extend(false);
    lift.setHeight(0.0);

    // Move intake
    double intake_angle = adjust(intake::getAngle, intake::setAngle, -1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), 0.0, 120.0);

    // Arm angle follows intake
    double arm_angle = -intake_angle;
    arm.setAngle(arm_angle);

    // Move to other mode?
    if (OI.joystick.getYButtonPressed())
      mode = Mode.NEAR;
  }

  private void handleNear()
  {
    // Fix: Intake, lift at bottom
    intake.setAngle(120.0);
    lift.setHeight(0.0);

    // Move arm angle
    double arm_angle = adjust(arm::getAngle, arm::setAngle, 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -180.0, 0.0);

    // Extend when arm is sticking outside the back,
    // or out front yet not too far up (to prevent topping)
    // This should keep arm pulled in while where "inside" the robot
    arm.extend(arm_angle < -170  ||  (arm_angle > -80.0  &&  arm_angle < -40.0));

    // Move to other mode?
    if (OI.joystick.getXButtonPressed())
      mode = Mode.INTAKE;
    if (OI.joystick.getYButtonPressed())
      mode = Mode.MID;
  }

  private void handleMid()
  {
    // Fix: Intake, lift at mid, arm in
    intake.setAngle(120.0);
    lift.setHeight(0.35);
    arm.extend(false);

    // Move arm angle
    adjust(arm::getAngle, arm::setAngle, 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -180.0, 0.0);

    // Move to other mode?
    if (OI.joystick.getXButtonPressed())
      mode = Mode.NEAR;
    if (OI.joystick.getYButtonPressed())
      mode = Mode.FAR;
  }

  private void handleFar()
  {
    // Fix: Intake, lift all up, arm out as soon as lift high enough
    intake.setAngle(120.0);
    lift.setHeight(0.7);
    arm.extend(lift.getHeight() > 0.4);

    // Move arm angle, but not too far out front
    adjust(arm::getAngle, arm::setAngle, 1.00*MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1), -180.0, -50.0);

    // Move to other mode?
    if (OI.joystick.getXButtonPressed())
      mode = Mode.MID;
  }
}
