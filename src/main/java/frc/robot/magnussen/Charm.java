// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import static java.lang.Math.IEEEremainder;
import static java.lang.Math.abs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  // Components that we handle
  private final Lift lift = new Lift();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final Grabber grabber = new Grabber();

  // TODO Tune basic arm, lift, intake moves to be faster
  // TODO Check every move/transition on actual robot.
  //      See how far arm needs to move out, lift up etc.
  //      when swapping arm/intake locations
  // TODO Can tables for near/mid/far be combined into one,
  //      so near/mid/far simply change the initial arm angle
  //      but can then move to any node level?

  /** Setpoints
   *  If we adjusted based on the current value from
   *  getAngle(), getHeight(), ...,
   *  we would amplify any regulation error:
   *  Lift a little too low -> next time we make that the setpoint,
   *  moving it even lower.
   *  So we track and then adjust the _setpoint_.
   */
  private double lift_setpoint, arm_setpoint, intake_setpoint;

  /** Command that awaits sufficient air pressure, once */
  private static class PressurizeCommand extends CommandBase
  {
    private static boolean pressurized = false;
    
    @Override
    public boolean isFinished()
    {
      if (! pressurized)
        pressurized = SmartDashboard.getNumber("Pressure", 0.0) > 80.0;
      return pressurized;
    }
  }

  /** Command that keeps setpoints where they are, forever, until cancelled */
  private class StayCommand extends CommandBase
  {
  }

  private class SetLiftCommand extends CommandBase
  {
    private double height;
    
    /** @param height Desired lift height, wait until within 3 cm */
    public SetLiftCommand(double height)
    {
      this.height = height;
    }

    @Override
    public void initialize()
    {
      lift_setpoint = height;
    }

    @Override
    public boolean isFinished()
    {
      return abs(lift_setpoint - lift.getHeight()) < 0.03;
    }
  }

  private class SetIntakeCommand extends CommandBase
  {
    private double desired;

    /** @param desired Desired intake angle, wait until within 5 degrees */
    public SetIntakeCommand(double desired)
    {
      this.desired = desired;
    }
  
    @Override
    public void initialize()
    {
      intake_setpoint = desired;
    }

    @Override
    public boolean isFinished()
    {
      return abs(IEEEremainder(desired - intake.getAngle(), 360)) < 5.0;
    }
  }

  private class SetIntakeSpinnerCommand extends InstantCommand
  {
    public SetIntakeSpinnerCommand(double voltage)
    {
      super(() -> intake.setSpinner(voltage));
    }
  }

  private class RetractArmCommand extends InstantCommand
  {
    public RetractArmCommand()
    {
      super(() -> arm.extend(false));
    }
  }

  private class ExtendArmCommand extends InstantCommand
  {
    public ExtendArmCommand()
    {
      super(() -> arm.extend(true));
    }
  }

  private class SetArmCommand extends CommandBase
  {
    private double desired;
    
    /** @param desired Desired arm angle, wait until within 5 degrees */
    public SetArmCommand(double desired)
    {
      this.desired = desired;
    }

    @Override
    public void initialize()
    {
      arm_setpoint = desired;
    }

    @Override
    public boolean isFinished()
    {
      return abs(IEEEremainder(desired - arm.getAngle(), 360)) < 5;
    }
  }

  private class InteractiveArmLiftExtendCommand extends CommandBase
  {
    protected final LookupTable table;

    /** @param table Table to use for arm angle, lift height, arm extension */
    public InteractiveArmLiftExtendCommand(LookupTable table)
    {
      this.table = table;
      addRequirements(Charm.this);
    }
    
    protected double getUserInput()
    {
      return MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1);
    }

    @Override
    public void execute()
    {
      // Lookup settings for adjusted arm angle
      Entry entry = table.lookup(arm_setpoint + getUserInput());
      // Use resulting values for arm(!), lift, extension
      arm_setpoint = entry.position;
      lift_setpoint = entry.values[0];
      arm.extend(entry.values[1] > 0.5);
    }
  }

  private static final LookupTable near_lookup = new LookupTable(
    new String[] { "Arm Angle", "Lift Height", "Extend" },
                           -90,          0.3,         0,
                           -82,          0,           0,
                           -70,          0,           1,
                             0,          0,           1);
  private class NearCommand extends InteractiveArmLiftExtendCommand
  {
    public NearCommand()
    {
      super(near_lookup);
    }

    @Override
    public void initialize()
    {
      SmartDashboard.putString("Mode", "Near");
      // TODO Preset arm, then allow interactive adjustment
      arm_setpoint = -74.0;
    }
  }

  private final LookupTable mid_lookup = new LookupTable(
   new String[] { "Arm Angle", "Lift Height", "Extend" },
                          -90,           0.5,   0,
                          -62,           0.55,  0,   
                          -45,           0.56,  0,
                            0,           0.3,   0);
  private class MidCommand extends InteractiveArmLiftExtendCommand
  {
    public MidCommand()
    {
      super(mid_lookup);
    }

    @Override
    public void initialize()
    {
      SmartDashboard.putString("Mode", "Mid");
      // TODO Preset arm, then allow interactive adjustment
      arm_setpoint = -60.0;
    }
  }

  private static final LookupTable far_lookup = new LookupTable(
   new String[] { "Arm Angle", "Lift Height", "Extend" },
                          -90,           0.7,   0,
                          -70,           0.7,   0, 
                          -25,           0.7,   1);
  private class FarCommand extends InteractiveArmLiftExtendCommand
  {
    public FarCommand()
    {
      super(far_lookup);
    }

    @Override
    public void initialize()
    {
      SmartDashboard.putString("Mode", "Far");
      // TODO Preset arm, then allow interactive adjustment
      arm_setpoint = -33.0;
    }
  }

  /** Move intake to capture cone and then transfer to grabber */
  private static final LookupTable cone_intake_arm_lookup = new LookupTable(
    new String[] { "Intake Angle", "Arm Angle", "Lift Height", "Extend" },
                               -2,         -88,          0.51,    0,
                               85,         -88,          0.51,    0,          
                               90,         -99,          0.25,    0); 
  private class InteractiveConeIntakeCommand  extends InteractiveArmLiftExtendCommand
  {
    public InteractiveConeIntakeCommand()
    {
      super(cone_intake_arm_lookup);
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

  private final static double INTAKE_IDLE_POS = 120;
  private final static double ARM_IDLE_POS = -90;

  private CommandBase idle()
  {
    // In autonomous, don't move anything because 'unfolding' could interfere with driving
    SequentialCommandGroup auto = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Auto")),
      new SetIntakeSpinnerCommand(0));
   
    // Unfold from intial setup or substation intake where arm is behind intake
    SequentialCommandGroup arm_behind_intake = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "ArmBehindIntake")),
      new SetIntakeSpinnerCommand(0),
      // This may be the initial unfold, so wait for air pressure
      new PressurizeCommand(),
      new RetractArmCommand(),
      new SetLiftCommand(0),
      // Move arm a little up/back
      new SetArmCommand(-155),
      // Move intake out to allow full arm movement
      new ParallelCommandGroup(new SetIntakeCommand(10.0),
                              // and move arm out once intake is out of the way
                               new WaitUntilCommand(() -> intake.getAngle() < 30)
                                   .andThen(new SetArmCommand(-10))
                               ),
      // Intake back in
      new ParallelCommandGroup(new SetIntakeCommand(INTAKE_IDLE_POS),
                               // Arm into idle position
                               new WaitUntilCommand(() -> intake.getAngle() > 100)
                                   .andThen(new SetArmCommand(ARM_IDLE_POS)))
    );

    // Intake is behind arm
    SequentialCommandGroup intake_behind_arm = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "IntakeBehindArm")),
      new SetIntakeSpinnerCommand(0),
      new RetractArmCommand(),
      // Intake and arm can freely move to idle position
      new ParallelCommandGroup(new SetIntakeCommand(INTAKE_IDLE_POS),
                               new SetArmCommand(ARM_IDLE_POS)),
      new SetLiftCommand(0));

    // Intake is out
    SequentialCommandGroup intake_out = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "IntakeOut")),
      new SetIntakeSpinnerCommand(0),
      // Intake is out, so can pull arm in, then lift up to get space for intake
      // Lift first, then arm in could mean arm out & lift up -> topple
      new RetractArmCommand(),
      new SetArmCommand(ARM_IDLE_POS),
      new SetLiftCommand(0.33),
      // Pull intake in
      new SetIntakeCommand(INTAKE_IDLE_POS),
      // Lift back down
      new SetLiftCommand(0));

    // Unknown/unhandled state
    SequentialCommandGroup unknown = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Idle??")),
      new SetIntakeSpinnerCommand(0));
        
    CommandBase selector = new ProxyCommand(() ->
    {
      if (DriverStation.isAutonomous())
        return auto;
      // Initial setup, also after substation intake: Arm back behind intake
      if (arm.getAngle() < -120 || Math.abs(arm.getAngle()) > 170)
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

  /** Assert that intake is in idle position, arm in front of it.
   *  If not, move everything to idle position
   */
  private class MakeSafeCommand extends CommandBase
  {
    private CommandBase sub_command;

    @Override
    public void initialize()
    {
      if (intake_setpoint == INTAKE_IDLE_POS  &&  arm_setpoint > -100)
        sub_command = null;
      else
      { // Only create idle commands when needed
        sub_command = idle();
        sub_command.initialize();
      }
    }

    @Override
    public void execute()
    {
      if (sub_command != null)
        sub_command.execute();
    }

    @Override
    public void end(boolean interrupted)
    {
      if (sub_command != null)
        sub_command.end(interrupted);
    }

    @Override
    public boolean isFinished()
    {
      return sub_command == null   ||  sub_command.isFinished();
    }
  }

  public void near()
  {
    new MakeSafeCommand().andThen(new NearCommand()).withName("Near").schedule();
  }

  public void mid()
  {
    new MakeSafeCommand().andThen(new MidCommand()).withName("Mid").schedule();
  }

  public void far()
  {
    new MakeSafeCommand().andThen(new FarCommand()).withName("Far").schedule();
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
      new MakeSafeCommand(),
      new ParallelCommandGroup
      (
        new SetLiftCommand(0.05),
        new SetArmCommand(-47)
      ),
      new SetIntakeCommand(6.0),
      new SetArmCommand(-85.0),
      new ExtendArmCommand(),
      new SetIntakeSpinnerCommand(Intake.SPINNER_VOLTAGE),
      // Call Grab*Commands via proxy, don't "require" it to allow return to 'off' ASAP
      new ProxyCommand(new GrabCubeCommand(grabber)),
      new RetractArmCommand(),
      new SetArmCommand(-47),
      new SetIntakeCommand(INTAKE_IDLE_POS),
      new SetArmCommand(ARM_IDLE_POS)
    );
    group.addRequirements(this);
    group.setName("IntakeCube");
    group.schedule();
  }

  public void intakeCone()
  {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "IntakeCone")),
      new MakeSafeCommand(),
      new SetLiftCommand(0.5),
      new SetIntakeCommand(0.0),
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
      new MakeSafeCommand(),
      // Lift out of the way ...
      new RetractArmCommand(),
      new SetLiftCommand(0.3),
      // .. for intake to move out and arm back in parallel
      new ParallelCommandGroup(new SetIntakeCommand(45),
                               new SetArmCommand(-120)),
      new SetLiftCommand(0),
      new ParallelCommandGroup(new SetIntakeCommand(80),
                               new SetArmCommand(-180)),
      new SetIntakeCommand(100),
      new ExtendArmCommand(),
      new ProxyCommand(() -> cube ? new GrabCubeCommand(grabber) : new GrabConeCommand(grabber)),
      // new RetractArmCommand(),
      new SetArmCommand(-170),
      new StayCommand()
    );
    group.addRequirements(this);
    group.setName("IntakeFromSubstation");
    group.schedule();
  }
}
