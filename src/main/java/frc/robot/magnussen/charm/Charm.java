// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import static java.lang.Math.abs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.magnussen.Arm;
import frc.robot.magnussen.GrabConeCommand;
import frc.robot.magnussen.GrabCubeCommand;
import frc.robot.magnussen.Grabber;
import frc.robot.magnussen.GrabberEjectCommand;
import frc.robot.magnussen.Intake;
import frc.robot.magnussen.Lift;
import frc.robot.magnussen.PressurizeCommand;
import frc.robot.magnussen.StayCommand;
import frc.robot.util.LookupTable;
import frc.robot.util.LookupTable.Entry;

/** Third time attempt at Arm/lift/grabber/intake
 * 
 *  When idle, the intake is folded back and the arm hangs down.
 * 
 *  Pro:
 *  Arm is in front of intake, ready to move arm & lift to score.
 *  Should a game piece drop from the grabber into the robot,
 *  it likely falls onto the lexan plate and can be forced out
 *  by driving backwards and/or spinning.
 * 
 *  Con:
 *  Taking game pieces in via intake first requires swapping positions
 *  so intake is before arm, then swapping back for idle position.
 * 
 *  Alternative: Arm is back at least -90 degree, intake in front at ~80 deg.
 * 
 *  Pro:
 *  Can simply drop intake down to take game pieces,
 *  also move arm back for substation intake.
 * 
 *  Con:
 *  Scoring requires swapping arm and intake position to get arm in front
 *  of intake, then swap back to store.
 *  Intake likely keeps a dropped gamepiece inside the robot.
 */
public class Charm extends SubsystemBase
{
  // Idle position of intake and arm
  final static double INTAKE_IDLE_POS = 120;
  final static double ARM_IDLE_POS = -90;

  // Components that we handle
  public final Lift lift = new Lift();
  public final Arm arm = new Arm();
  public final Intake intake = new Intake();
  public final Grabber grabber = new Grabber();

  /** Setpoints
   *  If we adjusted based on the current value from
   *  getAngle(), getHeight(), ...,
   *  we would amplify any regulation error:
   *  Lift a little too low -> next time we make that the setpoint,
   *  moving it even lower.
   *  So we track and then adjust the _setpoint_.
   */
  double lift_setpoint, arm_setpoint, intake_setpoint;

  /** Move intake to capture cone and then transfer to grabber */
  private static final LookupTable cone_intake_arm_lookup = new LookupTable(
    new String[] { "Intake Angle", "Arm Angle", "Lift Height", "Extend" },
                               -2,         -88,          0.51,    0,
                               85,         -88,          0.51,    0,          
                               90,         -99,          0.25,    0); 
  private class InteractiveConeIntakeCommand extends InteractiveArmLiftExtendCommand
  {
    public InteractiveConeIntakeCommand()
    {
      super(Charm.this, cone_intake_arm_lookup);
    }

    @Override
    public void initialize()
    {
      SmartDashboard.putString("Mode", "IntakeCone");
    }

    @Override
    public void execute()
    { // Contrary to InteractiveArmLiftExtendCommand, adjust _intake_
      Entry entry = cone_intake_arm_lookup.lookup(intake_setpoint + getUserInput());      
      // .. and then have arm, lift and extension follow
      intake_setpoint = entry.position;
      arm_setpoint = entry.values[0];
      lift_setpoint = entry.values[1];
      arm.extend(entry.values[2] > 0.5);
      
      // Grab/hold cone with intake.
      // Once cone is in grabber, stop grabbing it with the intake. In fact release it
      if (grabber.haveGamepiece())
        intake.setSpinner(2.0);
      else
        intake.setSpinner(-Intake.SPINNER_VOLTAGE);
    }
  }

  public Charm()
  {
    SmartDashboard.putString("Mode", "Init...");
    setDefaultCommand(idle().andThen(new StayCommand()).withName("StayIdle"));
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

  /** Move to INTAKE_IDLE_POS and ARM_IDLE_POS from any(?) initial setup */
  CommandBase idle()
  {
    // In autonomous, don't move anything because 'unfolding' could interfere with driving
    SequentialCommandGroup auto = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Auto")),
      new SetIntakeSpinnerCommand(this, 0));
   
    // Unfold from intial setup or substation intake where arm is behind intake
    SequentialCommandGroup arm_behind_intake = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "ArmBehindIntake")),
      new SetIntakeSpinnerCommand(this, 0),
      // This may be the initial unfold, so wait for air pressure
      new PressurizeCommand(),
      new RetractArmCommand(this),
      new SetLiftCommand(this, 0),
      // Move arm a little up/back
      new SetArmCommand(this, -155),
      // Move intake out to allow full arm movement
      new ParallelCommandGroup(new SetIntakeCommand(this, 10.0),
                              // and move arm out once intake is out of the way
                               new WaitUntilCommand(() -> intake.getAngle() < 30)
                                   .andThen(new SetArmCommand(this, -10))
                               ),
      // Intake back in
      new ParallelCommandGroup(new SetIntakeCommand(this, INTAKE_IDLE_POS),
                               // Arm into idle position
                               new WaitUntilCommand(() -> intake.getAngle() > 100)
                                   .andThen(new SetArmCommand(this, ARM_IDLE_POS)))
    );

    // Intake is behind arm
    SequentialCommandGroup intake_behind_arm = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "IntakeBehindArm")),
      new SetIntakeSpinnerCommand(this, 0),
      new RetractArmCommand(this),
      // Intake and arm can freely move to idle position
      new ParallelCommandGroup(new SetIntakeCommand(this, INTAKE_IDLE_POS),
                               new SetArmCommand(this, ARM_IDLE_POS)),
      new SetLiftCommand(this, 0));

    // Intake is out
    SequentialCommandGroup intake_out = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "IntakeOut")),
      new SetIntakeSpinnerCommand(this, 0),
      // Intake is out, so can pull arm in, then lift up to get space for intake
      // Lift first, then arm in could mean arm out & lift up -> topple
      new RetractArmCommand(this),
      new SetArmCommand(this, ARM_IDLE_POS),
      new SetLiftCommand(this, 0.33),
      // Pull intake in
      new SetIntakeCommand(this, INTAKE_IDLE_POS),
      // Lift back down
      new SetLiftCommand(this, 0));

    // Unknown/unhandled state
    SequentialCommandGroup unknown = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Idle??")),
      new SetIntakeSpinnerCommand(this, 0));
    
    CommandBase selector = new ProxyCommand(() ->
    {
      if (DriverStation.isAutonomous())
        return auto;
      // Initial setup, also after substation intake: Arm back behind intake
      // Consider wraparound when arm is way back at -120..-180 or 170..180
      if (abs(arm.getAngle()) >= 120)
        return arm_behind_intake;
      // Is arm out/in front of intake?
      if (intake.getAngle() > 100  &&  arm.getAngle() > -110)
        return intake_behind_arm;
      // Is intake 'out', not back and out of the way?
      if (intake.getAngle() < 110)
        return intake_out;
      // Cannot handle this scenario
      return unknown;
    });
    selector.addRequirements(this);

    return selector;
  }

  public void near()
  {
    new MakeSafeCommand(this).andThen(new NearCommand(this)).withName("Near").schedule();
  }

  public void mid()
  {
    new MakeSafeCommand(this).andThen(new MidCommand(this)).withName("Mid").schedule();
    // new MakeSafeCommand().andThen(new CombinedInteractiveCommand()).withName("Mid").schedule();
  }

  public void far()
  {
    new MakeSafeCommand(this).andThen(new FarCommand(this)).withName("Far").schedule();
  }

  public void eject()
  {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Eject")),
      new GrabberEjectCommand(grabber)
    );
    group.addRequirements(this);
    group.setName("Eject");
    group.schedule();
  }

  public void intakeCube()
  {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "IntakeCube")),
      new MakeSafeCommand(this),
      new ParallelCommandGroup(new SetLiftCommand(this, 0.05),
                               new SetArmCommand(this, -47)),
      new SetIntakeCommand(this, 6.0),
      new SetArmCommand(this, -85.0),
      new ExtendArmCommand(this),
      new SetIntakeSpinnerCommand(this, Intake.SPINNER_VOLTAGE),
      // Call Grab*Commands via proxy, don't "require" it to allow return to 'off' ASAP
      new ProxyCommand(new GrabCubeCommand(grabber)),
      new RetractArmCommand(this),
      new SetIntakeSpinnerCommand(this, 0),
      new SetArmCommand(this, -47),
      // TODO Next two in parallel, once speed of arm and intake have been optimized?
      new SetIntakeCommand(this, INTAKE_IDLE_POS),
      new SetArmCommand(this, ARM_IDLE_POS)
    );
    group.addRequirements(this);
    group.setName("IntakeCube");
    group.schedule();
  }

  public void intakeCone()
  {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "IntakeCone")),
      new MakeSafeCommand(this),
      new SetLiftCommand(this, 0.5),
      new SetIntakeCommand(this, 0.0),
      new ParallelDeadlineGroup(new ProxyCommand(new GrabConeCommand(grabber)),
                                new InteractiveConeIntakeCommand())
    );
    group.addRequirements(this);
    group.setName("IntakeCone");
    group.schedule();
  }

  public void intakeFromSubstation(boolean cube)
  {
    SequentialCommandGroup group = new SequentialCommandGroup
    (
      new InstantCommand(() -> SmartDashboard.putString("Mode", "SubIntake")),
      new PrintCommand("Intake from substation.."),
      new ProxyCommand(() ->
      { // Is arm already far enough back?
        if (arm.getAngle() <= -115)
          return new ParallelCommandGroup(new SetArmCommand(this, -180),
                                          new SetIntakeCommand(this, 100));
        else // If not, 'make safe' and start from there
          return new SequentialCommandGroup(new MakeSafeCommand(this),
                                            // Lift out of the way ...
                                            new RetractArmCommand(this),
                                            new SetLiftCommand(this, 0.3),
                                            // .. for intake to move out and arm back in parallel
                                            new ParallelCommandGroup(new SetIntakeCommand(this, 45),
                                                                    new SetArmCommand(this, -120)),
                                            // Lift down so arm can rotate out the back
                                            new SetLiftCommand(this, 0),
                                            new ParallelCommandGroup(new SetIntakeCommand(this, 80),
                                                                     new SetArmCommand(this, -180)),
                                            // With arm out back, intake can move further in
                                            new SetIntakeCommand(this, 100));
      }),
      // Either way, arm is now at -180 for item pickup, and intake around 100
      new ExtendArmCommand(this),
      new ProxyCommand(() -> cube ? new GrabCubeCommand(grabber) : new GrabConeCommand(grabber)),
      // Keep arm out the back, position for dropping cube in 'mid'
      new SetArmCommand(this, -200),
      // Stay that way until "eject", "near", "mid", "far", "xx intake" .. end this group 
      new StayCommand()
    );
    group.addRequirements(this);
    group.setName("IntakeFromSubstation");
    group.schedule();
  }
}
