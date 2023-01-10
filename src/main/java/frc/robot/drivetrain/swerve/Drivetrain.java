// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain.swerve;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Swerve module drive train */
public class Drivetrain extends SubsystemBase
{
  private final NetworkTableEntry nt_x = SmartDashboard.getEntry("X");
  private final NetworkTableEntry nt_y = SmartDashboard.getEntry("Y");
  private final NetworkTableEntry nt_heading = SmartDashboard.getEntry("Heading");

  NetworkTableEntry nt_xy_p = SmartDashboard.getEntry("P_XY");
  NetworkTableEntry nt_angle_p = SmartDashboard.getEntry("P_Angle");

  private final SwerveModule[] modules = new SwerveModule[]
  {
    new SwerveModule(0, SwerveModule.OFFSETS[0]),
    new SwerveModule(1, SwerveModule.OFFSETS[1]),
    new SwerveModule(2, SwerveModule.OFFSETS[2]),
    new SwerveModule(3, SwerveModule.OFFSETS[3])
  };

  /** Size of chassis which is not exactly a square,
   *  distance between swerve modules
   */
  final public static double WIDTH  = .64135;
  final public static double LENGTH = .61595;

  /** Don't rotate swerve module unless speed is at least this
   *  to avoid spinning in place
   */
  private static final double MINIMUM_SPEED_THRESHOLD = .05;

  /** Maximum drive speed to tame things down */
  private static final double MAXIMUM_SPEED = 2.0;

  /** Kinematics that translate chassis speed to module settings and vice versa */
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d( LENGTH / 2,  WIDTH / 2),
    new Translation2d( LENGTH / 2, -WIDTH / 2),
    new Translation2d(-LENGTH / 2, -WIDTH / 2),
    new Translation2d(-LENGTH / 2,  WIDTH / 2) );

  /** Gyro to measure robot's heading */
  private final GyroHelper gyro = new GyroHelper();

  /** Position tracker */
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
                                                                       gyro.getRotation(),
                                                                       getPositions());

  public Drivetrain()
  {
    // Publish command to reset position
    SmartDashboard.putData(new ResetPositionCommand(this));

    nt_xy_p.setDefaultDouble(1.0);
    nt_angle_p.setDefaultDouble(5.0);
  }

  /** Reset position tracker */
  public void reset()
  {
    gyro.reset();
    for (int i=0; i<modules.length; ++i)
      modules[i].resetPosition();
    odometry.resetPosition(Rotation2d.fromDegrees(0), getPositions(), new Pose2d());
  }

  /** @return Position of drivetrain on field (from odometry) */
  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }

  /** @return Heading of robot on field (relative to last "reset") */
  public double getHeading()
  {
    // Use odometry.getPoseMeters().getRotation()?
    return gyro.getRotation().getDegrees();
  }

  /** @return Positions of the swerve modules */
  private SwerveModulePosition[] getPositions()
  {
    final SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i=0; i<modules.length; ++i)
      positions[i] = modules[i].getPosition();
    return positions;
  }
  
  /** Drive all modules with same angle and speed */
  public void drive(double angle, double speed)
  {
    // modules[0].setSwerveModule(angle, speed);
    // modules[1].setSwerveModule(angle, speed);
    // modules[2].setSwerveModule(angle, speed);
    // modules[3].setSwerveModule(angle, speed);

    // for (int i=0; i<modules.length; ++i)
    //     modules[i].setSwerveModule(angle, speed);

    for (SwerveModule module : modules)
        module.setSwerveModule(angle, speed);
  }

  /** Swerve
   *  @param vx Speed in 'X' (forward/back) direction [m/s]
   *  @param vy Speed in 'Y' (left/right) direction [m/s]
   *  @param vr Speed for rotation [rad/s]
   *  @param center Center of rotation
   */
  public void swerve(double vx, double vy, double vr, Translation2d center)
  {
    // Translate desired chassis movement to settings of the 4 swerve modules
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, vr), center);

    for (int i=0; i<modules.length; ++i)
    {
      // Optimize module rotation
      states[i] = SwerveModuleState.optimize(states[i], modules[i].getCurrentAngle());

      // Actually moving? Then rotate as requested
      if (Math.abs(states[i].speedMetersPerSecond) < MINIMUM_SPEED_THRESHOLD)
          states[i] = new SwerveModuleState(0, modules[i].getCurrentAngle());
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAXIMUM_SPEED);

    for (int i=0; i<modules.length; ++i)
      modules[i].setSwerveModule(states[i].angle.getDegrees(),
                                 states[i].speedMetersPerSecond);
  }

  @Override
  public void periodic()
  {
    // Update and publish position
    odometry.update(gyro.getRotation(), getPositions());

    final Pose2d pose = odometry.getPoseMeters();
    nt_x.setDouble(pose.getX());
    nt_y.setDouble(pose.getY());
    nt_heading.setDouble(pose.getRotation().getDegrees());
  }

  /** @param trajectory Trajectory to follow
   *  @param end_angle Final heading angle
   *  @return Command that follows the trajectory
   */
  public Command createTrajectoryCommand(Trajectory trajectory, double end_angle)
  {
    // SwerveControllerCommand will basically send the speed at each point of the
    // trajectory to the serve modules, using many little helpers:
    // Controllers that correct for the x, y and angle to match the trajectory
    PIDController x_pid = new PIDController(nt_xy_p.getDouble(0), 0, 0);
    PIDController y_pid = new PIDController(nt_xy_p.getDouble(0), 0, 0);
    // Angle controller is 'profiled', allowing up to 30 deg/sec (and 30 deg/sec/sec acceleration) 
    ProfiledPIDController angle_pid = new ProfiledPIDController(
      nt_angle_p.getDouble(0), 0, 0,
      new TrapezoidProfile.Constraints(Math.toRadians(30), Math.toRadians(30)));
    // ..and 'continuous' because angle wraps around
    angle_pid.enableContinuousInput(-Math.PI, Math.PI);

    // Called by SwerveControllerCommand to check where we are
    Supplier<Pose2d> pose_getter = () -> odometry.getPoseMeters();
    // Called by SwerveControllerCommand to tell us what modules should do
    Consumer<SwerveModuleState[]> module_setter = states ->
    {
        for (int i=0; i<modules.length; ++i)
        {
          SwerveModuleState optimized = SwerveModuleState.optimize(states[i], modules[i].getCurrentAngle());
          modules[i].setSwerveModule(optimized.angle.getDegrees(),
                                     optimized.speedMetersPerSecond);
        }
    };
    // Called by SwerveControllerCommand to check at what angle we want to be
    Supplier<Rotation2d> desiredRotation = () -> Rotation2d.fromDegrees(end_angle);
    return new SwerveControllerCommand(trajectory, pose_getter, kinematics,
                                        x_pid, y_pid, angle_pid,
                                        desiredRotation, module_setter, this);
  }
}
