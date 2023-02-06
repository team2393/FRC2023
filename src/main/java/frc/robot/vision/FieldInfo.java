// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

/** Field information */
public class FieldInfo
{
  /** Database of all the april tags */
  public static final AprilTagFieldLayout april_tags;

  static
  {
    try
    {
      april_tags = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    }
    catch (Exception ex)
    {
      throw new IllegalStateException("Missing build-in field layout?!", ex);
    }
  }

  /** @param pose Pose
   *  @return Text representation
   */
  public static String format(Pose2d pose)
  {
    return String.format("X %3.1f Y %3.1f < %5.1f",
                         pose.getX(),
                         pose.getY(),
                         pose.getRotation().getDegrees());
  }

  /** IDs of tags on grids */
  public static final int[] GRID_TAGS = new int[] { 1, 2, 3, 6, 7, 8 };

  /** @param field_pos Position on field
   *  @return Pose of closest 'Grid' tag
   */
  public static Pose2d findClosestGridTag(Translation2d field_pos)
  {
    Pose2d nearest = null;
    double nearest_dist = Double.MAX_VALUE;
    for (int id : GRID_TAGS)
    {
      Pose3d tag_pose = april_tags.getTagPose(id).get();
      double dist = Math.hypot(tag_pose.getTranslation().getX() - field_pos.getX(),
                               tag_pose.getTranslation().getY() - field_pos.getY());
      if (dist < nearest_dist)
      {
        nearest_dist = dist;
        nearest = tag_pose.toPose2d();
      }
    }
    return nearest;
  }

  public static void main(String[] args)
  {
   System.err.println(format(findClosestGridTag(new Translation2d(1, 4)))); 
  }
}
