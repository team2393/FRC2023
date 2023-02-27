// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LookupTable;
import frc.robot.util.LookupTable.Entry;

/** Arm/lift/grabber/intake coordinator */
public class SecondAttempt extends SubsystemBase
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

  /** Base for command that uses the coordinator and shows its name as "Mode" */
  abstract private class CoordinatorCommand extends CommandBase
  {
    CoordinatorCommand()
    {
      addRequirements(SecondAttempt.this);
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

    protected double getUserInput()
    {
      // if (RobotBase.isReal())
        return MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1);
      // else
        // return (System.currentTimeMillis() / 6000) % 2 == 0 ? 1.0 : -1.0;
    }

  }

  /** Hold everything in a safe position inside the robot
   *  May invoke from any? configuration
   */
  private class StoreCommand extends CoordinatorCommand
  {
    @Override
    public void execute()
    {
      intake.setSpinner(0);
      arm.extend(false);

      if (grabber.haveGamepiece())
      {
        // TODO Higher in cone mode...
        lift_setpoint = 0.3;
      }
      else
        lift_setpoint = 0.0;

      // We want intake at 90 and arm at -110

      // Is arm 'outside'?
      if (arm.getAngle() > -90)
      { // Is intake out of the way?
        if (intake.getAngle() < 10.0)
            arm_setpoint = -110.0;
        else
        {
          // Move arm further outside
          arm_setpoint = -40;
          // Once there, put intake out and down into safe zone
          if (arm.getAngle() > -45)
            intake_setpoint = 5.0;
        }
      }
      else
      { // Arm is at least -90 deg "in"
        // Pull all the way in
        arm_setpoint = -110;
        if (arm.getAngle() < 100)
          intake_setpoint = 90.0;
      }
    }
  }

  /** Assuming components are stored,
   *  move arm 'out' with intake stored 'in'
   */
  private class UnStoreCommand extends CoordinatorCommand
  {
    private boolean done;

    @Override
    public void execute()
    {
      // Is intake 'in' and arm 'out'?
      done = intake.getAngle() > 80  &&  arm.getAngle() > -95;
      if (done)
        return;

      intake.setSpinner(0);
      arm.extend(false);

      if (grabber.haveGamepiece())
      {
        // TODO Higher in cone mode...
        lift_setpoint = 0.3;
      }
      else
        lift_setpoint = 0.0;

      // Starting with intake at 90 and arm at -110,
      // we want intake at 90 and arm at -45

      if (arm.getAngle() < -85.0)
      {
        // Move intake out/down to allow arm movement
        intake_setpoint = 5.0;
        if (intake.getAngle() < 10.0)
          arm_setpoint = -45.0;
      }
      else if (arm.getAngle() > -50.0)
        intake_setpoint = 90.0;

    }

    @Override
    public boolean isFinished()
    {
      return done;
    }
  }

  private class IntakeDownCommand extends CommandBase
  {
    @Override
    public void initialize()
    {
      intake_setpoint = 0;
    }

    @Override
    public boolean isFinished()
    {
      return intake.getAngle() < 10.0;
    }
  }

  private class IntakeUpCommand extends CommandBase
  {
    @Override
    public void initialize()
    {
      intake_setpoint = 90;
      intake.setSpinner(0);
    }

    @Override
    public boolean isFinished()
    {
      return intake.getAngle() > 80.0;
    }
  }

  private class SetLiftCommand extends CommandBase
  {
    private double height;
    
    public SetLiftCommand(double height)
    {
      this.height = height;
    }

    @Override
    public void execute()
    {
      lift_setpoint = height;
    }

    @Override
    public boolean isFinished()
    {
      return Math.abs(lift_setpoint - lift.getHeight()) < 0.1;
    }
  }

  private class SetArmCommand extends CommandBase
  {
    private double angle;
    
    public SetArmCommand(double angle)
    {
      this.angle = angle;
    }

    @Override
    public void execute()
    {
      arm_setpoint = angle;
    }

    @Override
    public boolean isFinished()
    {
      return Math.abs(Math.IEEEremainder(arm_setpoint - arm.getAngle(), 360)) < 5;
    }
  }

  /** Move intake to capture cone and then transfer to grabber */
  private static final LookupTable cone_intake_arm_lookup = new LookupTable(
    new String[] { "Intake Angle", "Arm Angle", "Lift Height", "Extend" },
                               -2,         -88,          0.51,    0,
                               65,         -88,          0.51,    0,
                               77,         -99,          0.25,    0); 
  private class InteractiveConeIntakeCommand extends CoordinatorCommand
  {
    private Timer timer = new Timer();

    @Override
    public void execute()
    {
      // Move intake
      Entry entry = cone_intake_arm_lookup.lookup(adjust(intake_setpoint, getUserInput()));      
      intake_setpoint = entry.position;
      arm_setpoint = entry.values[0];
      lift_setpoint = entry.values[1];
      arm.extend(entry.values[2] > 0.5);
      
      // If we have a cone, stop grabbing it with the intake. In fact release it
      if (grabber.haveGamepiece())
        intake.setSpinner(2.0);
      else
      {
        intake.setSpinner(-Intake.SPINNER_VOLTAGE);
        timer.restart();
      }
    }
  }

  private static final LookupTable near_lookup = new LookupTable(
    new String[] { "Arm Angle", "Lift Height", "Extend" },
                           -90,          0.3,         0,
                           -82,          0,           0,
                           -70,          0,           1,
                             0,          0,           1);
  private class NearCommand extends CoordinatorCommand
  {
    @Override
    public void execute()
    {
      // Intake in, lift at bottom
      intake_setpoint = 125.0;
      intake.setSpinner(0);
      
      // Move arm angle
      Entry entry = near_lookup.lookup(adjust(arm_setpoint, getUserInput()));
      arm_setpoint = entry.position;
      lift_setpoint = entry.values[0];
      arm.extend(entry.values[1] > 0.9);
    }
  }

  private final LookupTable mid_lookup = new LookupTable(
   new String[] { "Arm Angle", "Lift Height", "Extend" },
                          -90,           0.5,   0,
                          -62,           0.55,  0,   
                          -45,           0.56,  0,
                            0,           0.3,   0);
  private class MidCommand extends CoordinatorCommand
  {
    @Override
    public void execute()
    {
      // Intake in, lift at bottom
      intake_setpoint = 125.0;
      intake.setSpinner(0);
      
      // Move arm angle
      Entry entry = mid_lookup.lookup(adjust(arm_setpoint, getUserInput()));
      arm_setpoint = entry.position;
      lift_setpoint = entry.values[0];
      arm.extend(entry.values[1] > 0.5);
    }
  }

  private static final LookupTable far_lookup = new LookupTable(
   new String[] { "Arm Angle", "Lift Height", "Extend" },
                          -90,           0.7,   0,
                          -70,           0.7,   0, 
                          -25,           0.7,   1);
  private class FarCommand extends CoordinatorCommand
  {
    @Override
    public void execute()
    {
      // Intake in, lift at bottom
      intake_setpoint = 125.0;
      intake.setSpinner(0);
      
      // Move arm angle
      Entry entry = far_lookup.lookup(adjust(arm_setpoint, getUserInput()));
      arm_setpoint = entry.position;
      lift_setpoint = entry.values[0];
      arm.extend(entry.values[1] > 0.5);
    }
  }

  public SecondAttempt()
  {
    SmartDashboard.putString("Mode", "Init...");
    setDefaultCommand(new StoreCommand());
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
      // Keep in range -270..90
      if (arm_setpoint > 90.0)
        arm_setpoint -= 360.0;
      intake_setpoint = intake.getAngle();
    }
    else
    {
      lift.setHeight(lift_setpoint);
      arm.setAngle(arm_setpoint);
      intake.setAngle(intake_setpoint);
    }
  }
  
  /** @param value Current value
   *  @param rate Decrease/increase -1..1
   *  @return Adjusted value
   */
  private double adjust(double value, double rate)
  {
    value += rate;
    return value;
  }

  public void store()
  {
    new StoreCommand().schedule();
  }

  public void intakeCube()
  {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Intake Cube")),
      new PrintCommand("Starting cube intake.."),
      new IntakeDownCommand(),
      new SetLiftCommand(0.05),
      new SetArmCommand(-80.0),
      new InstantCommand(() -> arm.extend(true)),
      new InstantCommand(() -> intake.setSpinner(Intake.SPINNER_VOLTAGE)),
      new GrabCubeCommand(grabber),
      new InstantCommand(() -> arm.extend(false)),
      new SetLiftCommand(0.3),
      new SetArmCommand(-110.0),
      new IntakeUpCommand()
     );
    group.addRequirements(this);
    group.setName("IntakeCube");
    group.schedule();
  }

  public void intakeCone()
  {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Intake Cone")),
      new PrintCommand("Starting cone intake.."),
      new IntakeDownCommand(),
      new ParallelDeadlineGroup(new GrabConeCommand(grabber), new InteractiveConeIntakeCommand()),
      new SetLiftCommand(0.3)
     );
    group.addRequirements(this);
    group.setName("IntakeCone");
    group.schedule();
  }

  public void near()
  {
    new SequentialCommandGroup(
      new UnStoreCommand(),
      new SetArmCommand(-75.0),
      new NearCommand()).schedule();
  }

  public void mid()
  {
    new SequentialCommandGroup(
      new UnStoreCommand(),
      new MidCommand()).schedule();
  }

  public void far()
  {
    new SequentialCommandGroup(
      new UnStoreCommand(),
      new FarCommand()).schedule();
  }

  public void eject()
  {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Eject")),
      new PrintCommand("Ejecting.."),
      new GrabberEjectCommand(grabber)
    );
    group.addRequirements(this);
    group.setName("Eject");
    group.schedule();
  }
}
