// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swervelib.SwerveDrivetrain;

/** Helper for dealing with Limelight */
public class LimelightClient extends SubsystemBase
{
  private static class CameraInfo
  {
    /** ID of tag */
    public final int tag_id;

    /** Position of that tag on field  */
    public final Pose2d tag_position;

    /** Relative position of tag as seen by camera */
    public final Pose2d tag_view;

    public CameraInfo(int tag_id, Pose2d tag_position, Pose2d tag_view)
    {
      this.tag_id = tag_id;
      this.tag_position = tag_position;
      this.tag_view = tag_view;
    }

    @Override
    public String toString()
    {
      return String.format("#%d (%s) seen as %s",
                           tag_id,
                           FieldInfo.format(tag_position),
                           FieldInfo.format(tag_view));
    }
  }

  /** Data returned by Limelight when camera doesn't see a tag */
  private static final double[] NOTHING = new double[] { 0, 0, 0, 0, 0, 0 };

  /** How far is camera mounted 'forward' from the center of the robot? */
  public static double camera_forward = 0.32;

  /** How far is camera mounted 'left' from the center of the robot? */
  public static double camera_left = 0.0;
  
  /** NT entries read from camera */
  private final NetworkTableEntry nt_id, nt_data;
  
  /** NT entries updated with info obtained from camera */
  private final NetworkTableEntry nt_camera, nt_tagrel, nt_use_camera;
  
  private final SwerveDrivetrain drivetrain;

  /** Construct LimeLight client
   *  @param drivetrain Drivetrain where field position (odometry) is updated
   */
  public LimelightClient(SwerveDrivetrain drivetrain)
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    nt_id = table.getEntry("tid");
    nt_data = table.getEntry("targetpose_cameraspace");
    nt_camera = SmartDashboard.getEntry("CameraTagView");
    nt_tagrel = SmartDashboard.getEntry("PosFromTag");
    nt_use_camera = SmartDashboard.getEntry("UseCamera");
    
    this.drivetrain = drivetrain;
  }

  /** @return Camera info for closest Apriltag or null */
  private CameraInfo getCameraInfo()
  {
    // Does camera recognize a tag?
    int id = (int) nt_id.getInteger(-1);
    if (id < 0)
      return null;

    // Valid data?
    double [] data = nt_data.getDoubleArray(NOTHING);
    if (data.length != 6   ||  Arrays.equals(data, NOTHING))
      return null;
    
    // Is it a known tag?
    Optional<Pose3d> tag = FieldInfo.april_tags.getTagPose(id);
    if (tag.isEmpty())
      return null;
    
    // Camera reports "forward" (x) as "Z", "left (y)" as "-X",
    // and tag rotation as "Y" angle
    Pose2d tag_view = new Pose2d(data[2], -data[0], Rotation2d.fromDegrees(-data[4]));

    return new CameraInfo(id, tag.get().toPose2d(), tag_view);
  }

  /** @param info camera info, tag the camera saw and where
   *  @return Robot position on field
   */
  private static Pose2d computeRobotPose(CameraInfo info)
  {
    // Correct tag_view.translation by tag's rotation
    // to get the tag's view of the robot, looking straight from the tag
    Translation2d tags_robot_view = info.tag_view
                                        .getTranslation()
                                        .rotateBy(info.tag_view.getRotation().unaryMinus());

    // Get from tag's position to the robot's position 
    Translation2d camera_position = info.tag_position.getTranslation()
                                                     .plus(tags_robot_view);
    
    // Get from tag's view direction onto field to robot's,
    // considering that robots is pointed 180 degrees around to view the tag
    Rotation2d robot_heading = info.tag_position.getRotation()
                                                .minus(info.tag_view.getRotation())
                                                .plus(Rotation2d.fromDegrees(180));

    // Go from camera's position to center of robot
    Translation2d robot_position = camera_position.plus(new Translation2d(-camera_forward, -camera_left).rotateBy(robot_heading));

    return new Pose2d(robot_position, robot_heading);
  }

  @Override
  public void periodic()
  {
    CameraInfo info = getCameraInfo();
    if (info == null)
    {
      nt_camera.setString("?");
      nt_tagrel.setString("?");
    }
    else
    {
      // How camera saw the tag
      nt_camera.setString(String.format("%d @ %s",
                                        info.tag_id,
                                        FieldInfo.format(info.tag_view)));
                                        
      Pose2d camera_robot_pose = computeRobotPose(info);

      // Only use robot_position if it is within 1 meter of the current estimate?
      // Compare where we thought we were with where the camera tells us we are
      // Pose2d estimated_pose = drivetrain.getPose();
      // double dx = camera_robot_pose.getX() - estimated_pose.getX();
      // double dy = camera_robot_pose.getY() - estimated_pose.getY();
      // double distance = Math.sqrt(dx*dx + dy*dy);

      // Check how far the tag was from the camera
      double dx = info.tag_view.getX();
      double dy = info.tag_view.getY();
      double distance = Math.sqrt(dx*dx + dy*dy);

      boolean use_camera =distance < 1.0;
      nt_use_camera.setBoolean(use_camera);
      if (use_camera) 
      {
        // Update estimated field location
        // TODO Check https://docs.limelightvision.io/en/latest/networktables_api.html
        // tl - "The pipeline’s latency contribution (ms)"
        // " Add at least 11ms for image capture latency."
        double timestamp = Timer.getFPGATimestamp() - 0.011;
        drivetrain.updateLocationFromCamera(camera_robot_pose, timestamp);
      }

      // Robot pos relative to tag
      nt_tagrel.setString(FieldInfo.format(camera_robot_pose.relativeTo(info.tag_position)));
    }
  }

  /** Offline demo */
  public static void main(String[] args)
  {
    CameraInfo info = new CameraInfo(6,
                                     new Pose2d(1, 4.4, Rotation2d.fromDegrees(0)),
                                     new Pose2d(1.4, 0, Rotation2d.fromDegrees(-20)));
    
    System.out.println(info);

    Pose2d robot_pose = computeRobotPose(info);

    System.out.println(FieldInfo.format(robot_pose));
  }
}
