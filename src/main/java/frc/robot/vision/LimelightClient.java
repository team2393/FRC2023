// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Helper for dealing with Limelight
 *
 *  TODO Test with actual camera
 *  TODO Add MedianFilter(3), see last year
 */
public class LimelightClient extends SubsystemBase
{
  private static final String LIMELIGHT_NAME = "limelight-front";
  private static final String POSE_DATA = "targetpose_cameraspace";
  private static final double[] NOTHING = new double[] { 0, 0, 0, 0, 0, 0 };
  private final NetworkTableEntry nt_data;

  public LimelightClient()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
    nt_data = table.getEntry(POSE_DATA);
  }

  /** @return Nearest Apriltag pose in robot coordinates or null */
  public Pose3d readTargetPose()
  {
    double [] data = nt_data.getDoubleArray(NOTHING);
    if (data.length != 6   ||  Arrays.equals(data, NOTHING))
      return null;
    return new Pose3d(data[2], -data[0], -data[1],
                      new Rotation3d(0.0, 0.0, -Math.toRadians(data[4])));
  }

  @Override
  public void periodic()
  {
    Pose3d nearest = readTargetPose();
    String info;
    if (nearest == null)
      info = "<unknown>";
    else
      info = String.format("X %.2f, Y %.2f, Z %.2f m at %.1f degrees\n",
                      nearest.getTranslation().getX(),
                      nearest.getTranslation().getY(),
                      nearest.getTranslation().getZ(),
                      Math.toDegrees(nearest.getRotation().getZ()));
    SmartDashboard.putString("Target", info);
  }
}
