// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Helper for dealing with camera vs. robot coordinate systems */
public class CameraCoords
{  
  // AprilTagPoseEstimator coordinate system:
  // Z = distance from camera
  // X = "right"
  // Y = "down"
  //      Z
  //      |
  //      |
  //     (Y) ----> X
  //   |CAMERA|
  //
  // Rotate 90 deg around X axis:
  //      Y
  //      |
  //      |
  //      Z ----> X
  //
  // Rotate 90 deg around Z:
  //      X
  //      |
  //      |
  // Y<---Z
  //   |ROBOT|
  //
  // ==>  Robot coordinate system:
  // X = "forward"
  // Y = "left"
  // Z = "up"
  //
  // To translate from the camera to the robot system, reverse the rotations: X @ -90 deg X, Z @ -90 deg
  public static final Rotation3d CAMERA_TO_ROBOT_AXES = new Rotation3d(-Math.PI/2, 0, -Math.PI/2);

  /** @param rotation Rotation around X, Y, Z axes
   *  @return Text representation in degrees
   */
  public static String rotationInfo(Rotation3d rotation)
  {
    return String.format("X %.1f deg, Y %.1f deg, Z %.1f deg",
                         Math.toDegrees(rotation.getX()),
                         Math.toDegrees(rotation.getY()),
                         Math.toDegrees(rotation.getZ()));
  } 

  /** @param camera_tag_position Tag position as viewed by the camera
   *  @return Position of tag relative to robot
   */
  static public Translation3d cameraTagToRobotCoordinates(Translation3d camera_tag_position)
  {
    return new Translation3d(camera_tag_position.getZ(), -camera_tag_position.getX(), -camera_tag_position.getY());
  }

  // Demo of converting april tag poses to robot/field cooords
  public static void main(String[] args)
  {
    System.out.println("Transform from camera coords to robot coords: " +
                       rotationInfo(CAMERA_TO_ROBOT_AXES));
    // 3m away, 2m left, 1m up in camera axes
    Translation3d camera_tag_view = new Translation3d(-2, -1, 3);
    System.out.println("Camera tag info    : " + camera_tag_view);

    // Expect x=3, y=2, z=1
    Translation3d robot_tag_view = camera_tag_view.rotateBy(CAMERA_TO_ROBOT_AXES);
    System.out.println(" ==> Robot tag info: " + robot_tag_view);
    robot_tag_view = cameraTagToRobotCoordinates(camera_tag_view);
    System.out.println("Simpler?  ==>        " + robot_tag_view);

    // As seen by the robot, assume tag is rotated 
    Rotation3d tag_rotation = new Rotation3d(0, Math.toRadians(-30), 0);
    System.out.println("Camera's view of tag angles   : " + rotationInfo(tag_rotation));
    // Angle of the camera around the 'up' axis
    // 0 degrees = it's facing us straight on
    double camera_angle = Math.toDegrees(-tag_rotation.getY());
    System.out.println("Camera rotation seen by robot : " + camera_angle + " degrees around 'up' axis (Z)");

    System.out.println();

    // We might have a map where we can fetch the Pose3d for each tag based on the tag ID:
    Pose3d tag_info = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    // .. sbut we only really care about the 2d (X, Y, heading):
    Pose2d tag_pose = new Pose2d(tag_info.getTranslation().toTranslation2d(),
                                 tag_info.getRotation().toRotation2d());
    System.out.println("Assume tag at " + tag_pose);
    
    // Based on where the tag is on the field
    // and how we see it from the robot,
    // compute absolute robot position on the field.
    // Start with the location of the tag,
    // move by how robot sees the tag,
    // rotate by angle of camera
    Translation2d robot_pos = tag_pose.getTranslation()
                                      .plus(robot_tag_view.toTranslation2d())
                                      .rotateBy(Rotation2d.fromDegrees(camera_angle));
    // Robot is looking 180 from the camera
    double robot_heading = Math.IEEEremainder(
      tag_pose.getRotation().getDegrees() + 180 + camera_angle, 360);
    System.out.println("===> Robot at " + robot_pos + ", heading " + robot_heading);    
  }
}
