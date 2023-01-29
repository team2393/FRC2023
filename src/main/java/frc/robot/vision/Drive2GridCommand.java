package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swervelib.SwerveDrivetrain;
import frc.robot.swervelib.SwerveToPositionCommand;

public class Drive2GridCommand extends CommandBase
{
  private SwerveDrivetrain drivetrain;
  
  public Drive2GridCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize()
  {
    Pose2d robot_pose = drivetrain.getPose();
    Pose2d grid_pose = FieldInfo.findClosestGridTag(robot_pose.getTranslation());
    if (grid_pose == null)
    {
      System.out.println("Cannot locate nearby grid");
      return;
    }

    System.out.println("Nearest grid: " + grid_pose);

    // Drive to 1.0m in front of that grid, with nose of robot towards the grid
    Pose2d target = new Pose2d(
      grid_pose.getTranslation().plus(new Translation2d(1.0, 0).rotateBy(grid_pose.getRotation())),
      grid_pose.getRotation().plus(Rotation2d.fromDegrees(180)));

    System.out.println("--> run to  " + target);

    // Schedule move to that position, which cancels this command
    new SwerveToPositionCommand(drivetrain, target).schedule();
  }
}
