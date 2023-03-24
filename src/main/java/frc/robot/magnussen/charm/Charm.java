// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import static java.lang.Math.abs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
 *  .. without intake
 */
public class Charm extends SubsystemBase
{
  // Idle position of arm
  final static double ARM_IDLE_POS = -90;

  // Components that we handle
  public final Lift lift = new Lift();
  public final Arm arm = new Arm();
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

  /** Simulation/Visualization */
  private final Mechanism2d mechanism;
  private final MechanismLigament2d mech_lift, mech_arm, mech_intake;

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
    }
  }

  public Charm()
  {
    SmartDashboard.putString("Mode", "Init...");
    setDefaultCommand(idle().andThen(new StayCommand()).withName("StayIdle"));

    // Create "Mechanism" that can be displays in sim GUI:
    // NetworkTables -> Smart Dashboard -> Mechanism
    mechanism = new Mechanism2d(0.8, 1.5, new Color8Bit(Color.kWhite));

    MechanismRoot2d lift_root = mechanism.getRoot("lift_root", 0.18, 0);
    mech_lift = lift_root.append(new MechanismLigament2d("lift", 0.9, 60, 10, new Color8Bit(Color.kRed)));
    mech_arm = mech_lift.append(new MechanismLigament2d("arm", 0.4, 0, 10, new Color8Bit(Color.kGreen)));
    mech_arm.append(new MechanismLigament2d("grab1", 0.2, 45, 10, new Color8Bit(Color.kBrown)));
    mech_arm.append(new MechanismLigament2d("grab2", 0.2, -45, 10, new Color8Bit(Color.kBrown)));

    MechanismRoot2d intake_root = mechanism.getRoot("intake_root", 0.2, 0);
    mech_intake = intake_root.append(new MechanismLigament2d("intake", 0.3, 0, 10, new Color8Bit(Color.kBlue)));

    SmartDashboard.putData("Mechanism", mechanism);
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
    }
    else
    {
      lift.setHeight(lift_setpoint);
      arm.setAngle(arm_setpoint);
    }

    mech_lift.setLength(0.9 + lift.getHeight());
    mech_arm.setAngle(arm.getAngle() - mech_lift.getAngle());
    mech_arm.setLength(arm.isExtended() ? (0.4+0.2) : 0.4);
  }

  /** Move to INTAKE_IDLE_POS and ARM_IDLE_POS from any(?) initial setup */
  CommandBase idle()
  {
    // In autonomous, don't move anything
    SequentialCommandGroup auto = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "Auto")));
   
    // Move all to idle posittion
    SequentialCommandGroup make_idle = new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("Mode", "MakeIdle")),
      new RetractArmCommand(this),
      new SetArmCommand(this, ARM_IDLE_POS),
      new SetLiftCommand(this, 0));
       
    CommandBase selector = new ProxyCommand(() ->
    {
      if (DriverStation.isAutonomous())
        return auto;
      else
       return make_idle;
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

  public void intakeFromSubstation(boolean cube)
  {
    SequentialCommandGroup group = new SequentialCommandGroup
    (
      new InstantCommand(() -> SmartDashboard.putString("Mode", "SubIntake")),
      new PrintCommand("Intake from substation.."),
      new ProxyCommand(() ->
      { // Is arm already far enough back?
        if (arm.getAngle() <= -115)
          return new SetArmCommand(this, -180);
        else // If not, 'make safe' and start from there
          return new SequentialCommandGroup(new MakeSafeCommand(this),
                                            // Lift down so arm can rotate out the back
                                            new SetLiftCommand(this, 0),
                                            new SetArmCommand(this, -180));
      }),
      // Either way, arm is now at -180 for item pickup
      new ExtendArmCommand(this),
      new ProxyCommand(() -> cube ? new GrabCubeCommand(grabber) : new GrabConeCommand(grabber)),
      new RetractArmCommand(this),
      new WaitCommand(1.5),
      // Put arm inside robot
      new SetArmCommand(this, -110),
      // Stay that way until "eject", "near", "mid", "far", "xx intake" .. end this group 
      new StayCommand()
    );
    group.addRequirements(this);
    group.setName("IntakeFromSubstation");
    group.schedule();
  }

  public void recoverCube()
  {
    SequentialCommandGroup group = new SequentialCommandGroup
    (
      new MakeSafeCommand(this),
      new RetractArmCommand(this),
      new SetArmCommand(this, -108),
      new SetLiftCommand(this, 0.10),
      new ParallelDeadlineGroup(new ProxyCommand(new GrabCubeCommand(grabber)), new ExtendArmCommand(this)),
      new RetractArmCommand(this),
      new WaitCommand(1.0),
      new SetLiftCommand(this, 0),
      new StayCommand()
    );
    group.addRequirements(this);
    group.setName("RecoverCube");
    group.schedule();
  }
}
