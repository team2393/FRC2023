// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
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
  //
  // ==>  Robot coordinate system:
  // X = "forward"
  // Y = "left"
  // Z = "up"
  //
  // To translate a pose or rotation from the camera system
  // to the robot system, reverse the rotations: X @ -90 deg X, Z @ -90 deg
  final static private double DEG_90 = Math.PI/2;
  final static public Rotation3d CAMERA_TO_ROBOT_AXES = new Rotation3d(-DEG_90, 0, -DEG_90);

  /** @param tag_rotation Rotation of tag's Pose from AprilTagPoseEstimator
   *  @return Angle of tag's twist around 'up' axis (Z for robot system)
   */
  static public double getTwist(Rotation3d tag_rotation)
  {
    // We only care about the "up" axis
    return Math.toDegrees(-tag_rotation.getY());
  } 

  static public String rotationInfo(Rotation3d rotation)
  {
    return String.format("X @ %.1f, Y @ %.1f, Z @ %.1f",
                         Math.toDegrees(rotation.getX()),
                         Math.toDegrees(rotation.getY()),
                         Math.toDegrees(rotation.getZ()));
  } 

  // Demo of converting april tag poses to robot/field cooords
  public static void main(String[] args)
  {
    System.out.println("Transform: " + rotationInfo(CAMERA_TO_ROBOT_AXES));
    // 3m away, 2m left, 1m up in camera axes
    Translation3d tag = new Translation3d(-2, -1, 3);
    System.out.println("Camera tag info    : " + tag);

    // Expect x=3, y=2, z=1
    System.out.println(" ==> Robot tag info: " + tag.rotateBy(CAMERA_TO_ROBOT_AXES));

    Rotation3d tag_rotation = new Rotation3d(0, Math.toRadians(-10), 0);
    System.out.println("Tag twist as seen by camera: " + rotationInfo(tag_rotation));
    System.out.println("Tag twist as seen by robot : " + getTwist(tag_rotation) + " degrees around 'up' axis (Z)");
  }
}
