package frc.robot.swervelib;
// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Swerve module drive train */
abstract public class SwerveDrivetrain extends SubsystemBase
{
  /** Don't rotate swerve module unless speed is at least this
   *  to avoid spinning in place
   */
  public static double MINIMUM_SPEED_THRESHOLD = .05;

  /** Maximum drive speed to tame things down */
  public static double MAXIMUM_SPEED = 2.0;

  /** Position on field */
  private final NetworkTableEntry nt_x = SmartDashboard.getEntry("X");
  private final NetworkTableEntry nt_y = SmartDashboard.getEntry("Y");
  private final NetworkTableEntry nt_heading = SmartDashboard.getEntry("Heading");
  private final Field2d field = new Field2d();

  /** Trajectory follower P gains */
  private final NetworkTableEntry nt_xy_p = SmartDashboard.getEntry("Traj XY P");
  private final NetworkTableEntry nt_angle_p = SmartDashboard.getEntry("Traj Angle P");

  /** Rectangle where modules are on the corners */
  private final double width, length;

  /** Front left, front right, back right, back left module */
  private final SwerveModule[] modules;

  /** Zero offset for gyro in degrees */
  private double zero_heading = 0.0;

  /** Simulated gyro angle in degrees */
  private double simulated_heading = 0.0;

  /** Kinematics that translate chassis speed to module settings and vice versa */
  private final SwerveDriveKinematics kinematics;

  /** Position tracker */
  private final SwerveDriveOdometry odometry;

  /** @param width Width of the rectangle where modules are on corners in meters
   *  @param length Length of that rectangle in meters
   *  @param modules Front left, front right, back right, back left module
   */
  public SwerveDrivetrain(double width, double length, SwerveModule[] modules)
  {
    this.width = width;
    this.length = length;
    this.modules = modules;
    
    kinematics = new SwerveDriveKinematics(new Translation2d( length / 2,  width / 2),
                                           new Translation2d( length / 2, -width / 2),
                                           new Translation2d(-length / 2, -width / 2),
                                           new Translation2d(-length / 2,  width / 2) );

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), getPositions());

    // Publish command to reset position
    SmartDashboard.putData(new ResetPositionCommand(this));

    // Publish field
    SmartDashboard.putData(field);

    nt_xy_p.setDefaultDouble(1.0);
    nt_angle_p.setDefaultDouble(5.0);

    // When no other command uses the drivetrain, stay put with modules pointed to 0.0
    setDefaultCommand(new StayPutCommand(this, 0.0));
  }

  /** @return Width of the rectangle where modules are on corners in meters */
  public double getWidth()
  {
    return width;
  }

  /** @return Length of the rectangle where modules are on corners in meters */
  public double getLength()
  {
    return length;
  }

  /** Reset position tracker */
  public void reset()
  {
    zero_heading = getRawHeading();
    simulated_heading = 0.0;
    for (int i=0; i<modules.length; ++i)
      modules[i].resetPosition();
    odometry.resetPosition(getHeading(), getPositions(), new Pose2d());
  }

  /** @return Heading of gyro in degrees, not corrected for zero heading */
  abstract public double getRawHeading();

  /** @return Heading of robot on field (relative to last "reset") */
  public Rotation2d getHeading()
  {
    if (RobotBase.isSimulation())
      return Rotation2d.fromDegrees(simulated_heading);
    return Rotation2d.fromDegrees(getRawHeading() - zero_heading);
  }

  /** @return Positions of the swerve modules */
  private SwerveModulePosition[] getPositions()
  {
    final SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i=0; i<modules.length; ++i)
      positions[i] = modules[i].getPosition();
    return positions;
  }

  /** @return Position of drivetrain on field (from odometry) */
  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }

  /** Drive all modules with same angle and speed
   *  @param angle Swerve module angle in degrees
   *  @param speed Swerve module speed in meters per second
   */
  public void drive(double angle, double speed)
  {
    for (SwerveModule module : modules)
        module.drive(angle, speed);
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
      states[i] = SwerveModuleState.optimize(states[i], modules[i].getAngle());

      // Actually moving? Then rotate as requested
      if (Math.abs(states[i].speedMetersPerSecond) < MINIMUM_SPEED_THRESHOLD)
          states[i] = new SwerveModuleState(0, modules[i].getAngle());
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAXIMUM_SPEED);

    for (int i=0; i<modules.length; ++i)
      modules[i].drive(states[i].angle.getDegrees(),
                       states[i].speedMetersPerSecond);
    
    if (RobotBase.isSimulation())
    {
      double adjusted_vr = Math.toDegrees(kinematics.toChassisSpeeds(states).omegaRadiansPerSecond);
      simulated_heading += adjusted_vr * TimedRobot.kDefaultPeriod;
    }
  }

  @Override
  public void periodic()
  {
    // Update and publish position
    odometry.update(getHeading(), getPositions());

    final Pose2d pose = odometry.getPoseMeters();
    nt_x.setDouble(pose.getX());
    nt_y.setDouble(pose.getY());
    nt_heading.setDouble(pose.getRotation().getDegrees());

    // Move '0,0' to other position on field to get better looking start position
    field.setRobotPose(new Pose2d(pose.getTranslation()
                                      .rotateBy(Rotation2d.fromDegrees(90))
                                      .plus(new Translation2d(6, 4)),
                                  getHeading().plus(Rotation2d.fromDegrees(90))));
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
          SwerveModuleState optimized = SwerveModuleState.optimize(states[i], modules[i].getAngle());
          modules[i].drive(optimized.angle.getDegrees(),
                           optimized.speedMetersPerSecond);
        }
        double vr = Math.toDegrees(kinematics.toChassisSpeeds(states).omegaRadiansPerSecond);
        simulated_heading += vr * TimedRobot.kDefaultPeriod;
    };
    // Called by SwerveControllerCommand to check at what angle we want to be
    Supplier<Rotation2d> desiredRotation = () -> Rotation2d.fromDegrees(end_angle);
    return new SwerveControllerCommand(trajectory, pose_getter, kinematics,
                                        x_pid, y_pid, angle_pid,
                                        desiredRotation, module_setter, this);
  }
}
