// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Swerve module drive train */
public class Drivetrain extends SubsystemBase
{
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
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d( LENGTH / 2,  WIDTH / 2),
    new Translation2d( LENGTH / 2, -WIDTH / 2),
    new Translation2d(-LENGTH / 2, -WIDTH / 2),
    new Translation2d(-LENGTH / 2,  WIDTH / 2) );

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
}
