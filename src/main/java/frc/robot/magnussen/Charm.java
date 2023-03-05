// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import static java.lang.Math.IEEEremainder;
import static java.lang.Math.abs;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.LookupTable;
import frc.robot.util.LookupTable.Entry;

/** Third time attempt at Arm/lift/grabber/intake
 * 
 *  When idle, the intake is folded back and the arm hangs down.
 * 
 *  Pro:
 *  Arm is in front of intake, ready to move arm/lift to score.
 *  Should a game piece drop from the grabber into the robot,
 *  it likely falls onto the lexan plate and can be forced out
 *  by driving backwards or spinning.
 * 
 *  Con:
 *  Taking game pieces in, i.e., moving intake out first requires
 *  swapping arm and intake position
 * 
 *  Alternative: Arm is back at least -100 degree, intake in front ~80 deg.
 * 
 *  Pro:
 *  Can simply drop intake down to take game pieces,
 *  also move arm back for substation intake.
 * 
 *  Con:
 *  Scoring requires swapping arm and intake position to get arm in front
 *  of intake, then swap back to store.
 *  Intake likely keeps a dropped gamepiece inside the robot.
 *  
 */
public class Charm extends SubsystemBase
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

  /** Do we have enough air pressure? */
  private static boolean pressurized = false;

  /** Command that awaits sufficient air pressure, once */
  private class PressurizeCommand extends CommandBase
  {
    @Override
    public void initialize()
    {
      if (! pressurized)
        System.out.println("Waiting for air pressure");
    }
    @Override
    public void execute()
    {
      // If already pressurized, once: fine. If never, check
      if (! pressurized)
        pressurized = SmartDashboard.getNumber("Pressure", 0.0) > 80.0;
    }

    @Override
    public boolean isFinished()
    {
      return pressurized;
    }
  }


  /** Command that keeps setpoints where they are */
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
    public void execute()
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
    private final LookupTable table;

    /** @param table Table to use for arm angle, lift height, arm extension */
    public InteractiveArmLiftExtendCommand(LookupTable table)
    {
      this.table = table;
      addRequirements(Charm.this);
    }
    
    protected double getUserInput()
    {
      // if (RobotBase.isReal())
        return MathUtil.applyDeadband(OI.getCombinedTriggerValue(), 0.1);
      // else
        // return (System.currentTimeMillis() / 6000) % 2 == 0 ? 1.0 : -1.0;
    }

    @Override
    public void execute()
    {
      // Move arm angle
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
      arm_setpoint = -85.0;
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
      arm_setpoint = -76.0;
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
      arm_setpoint = -5.0;
    }
  }

  public Charm()
  {
    SmartDashboard.putString("Mode", "Init...");
    setDefaultCommand(idle());
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

  private CommandBase idle()
  {
    final double INTAKE_IDLE_POS = 120;
    final double ARM_IDLE_POS = -90;

    SequentialCommandGroup auto = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Auto")),
      new StayCommand());

    // Unfold from intial setup
    SequentialCommandGroup initial_unfold = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Unfold")),
      new PressurizeCommand(),
      new RetractArmCommand(),
      new SetLiftCommand(0),
      // Move arm a little up/back so intake can move
      new SetArmCommand(-155),
      // Move intake out to allow full arm movement
      new SetIntakeCommand(10.0),
      // Arm out
      new SetArmCommand(-10),
      // Intake back in
      new SetIntakeCommand(INTAKE_IDLE_POS),
      // Arm into idle position
      new SetArmCommand(ARM_IDLE_POS),
      new StayCommand());

    // Intake is behind arm
    SequentialCommandGroup intake_behind_arm = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Idle")),
      new RetractArmCommand(),
      // Intake and arm can freely move to idle position
      new ParallelCommandGroup(new SetIntakeCommand(INTAKE_IDLE_POS),
                               new SetArmCommand(ARM_IDLE_POS)),
      new SetLiftCommand(0),
      new StayCommand());

    // Intake is out
    SequentialCommandGroup intake_out = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "IdleIntake")),
      // Clear area for intake
      new SetArmCommand(-90.0),
      new RetractArmCommand(),
      new SetLiftCommand(0.5),
      new SetIntakeCommand(INTAKE_IDLE_POS),
      new SetArmCommand(ARM_IDLE_POS),
      new SetLiftCommand(0),
      new StayCommand());

    // Unknown/unhandled state
    SequentialCommandGroup unknown = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Idle??")),
      new StayCommand());
        
    Map<Object, Command> options = Map.of("Auto", auto,
                                          "Initial", initial_unfold,
                                          "IntakeBehindArm", intake_behind_arm,
                                          "IntakeOut", intake_out,
                                          "Unknown", unknown);
    Supplier<Object> decider = () ->
    {
      if (DriverStation.isAutonomous())
        return "Auto";
      // Initial setup (also after substation intake): Arm back behind intake
      if (arm.getAngle() < -120)
        return "Initial";
      // Is arm out/in front of intake?
      if (intake.getAngle() > 100  &&  arm.getAngle() > -110)
        return "IntakeBehindArm";
      if (intake.getAngle() < 90)
        return "IntakeOut";
      // Cannot handle this scenario
      return "Unknown";
    };
    SelectCommand selector = new SelectCommand(options, decider);
    selector.addRequirements(this);

    return selector;
  }

  public void near()
  {
    new NearCommand().schedule();
  }

  public void mid()
  {
    new MidCommand().schedule();
  }

  public void far()
  {
    new FarCommand().schedule();
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
      new SetLiftCommand(0.5),
      new SetIntakeCommand(6.0),
      new SetLiftCommand(0.05),
      new SetArmCommand(-85.0),
      new ExtendArmCommand(),
      new InstantCommand(() -> intake.setSpinner(Intake.SPINNER_VOLTAGE)),
      new GrabCubeCommand(grabber)
    );
    group.addRequirements(this);
    group.setName("IntakeCube");
    group.schedule();
  }

  public void intakeFromSubstation(boolean cube)
  {
    SequentialCommandGroup group = new SequentialCommandGroup
    (
      new InstantCommand(() -> SmartDashboard.putString("Mode", "SubIntake")),
      new PrintCommand("Intake from substation.."),
      // Lift out of the way ...
      new RetractArmCommand(),
      new SetLiftCommand(0.5),
      // .. for intake to move out and arm back in parallel
      new ParallelCommandGroup(new SetIntakeCommand(45),
                               new SetArmCommand(-120)),
      new SetLiftCommand(0),
      new ParallelCommandGroup(new SetIntakeCommand(90),
                               new SetArmCommand(-180)),
      new ExtendArmCommand(),
      new ProxyCommand(() -> cube ? new GrabCubeCommand(grabber) : new GrabConeCommand(grabber)),
      new RetractArmCommand()
    );
    group.addRequirements(this);
    group.setName("IntakeFromSubstation");
    group.schedule();
  }

}
