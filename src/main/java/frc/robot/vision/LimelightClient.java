// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.nio.file.Files;
import java.nio.file.Path;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.CANSparkMax.ExternalFollower;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Helper for dealing with Limelight
 *
 *  Fetch network table entry "limelight-front", "json"
 *  and decode the "t6t_rs"
 *
 *  TODO Test with actual camera
 *  TODO Add MedianFilter(3), see last year
 */
public class LimelightClient
{
  private static final String LIMELIGHT_NAME = "limelight-front";
  private static final String POSE_DATA = "t6t_rs";
  private static final  ObjectMapper mapper = new ObjectMapper();
  private final NetworkTableEntry nt_json;

  public LimelightClient()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
    nt_json = table.getEntry("json");
  }

  /** @return Apriltag detection info from Limelight in JSON format */
  public String getJSON()
  {
    return nt_json.getString("json");
  }

  /** @param json Apriltag detection info from Limelight
   *  @return Nearest Apriltag pose in robot coordinates
   *  @throws Exception on error
   */
  public static Pose3d decode(String json) throws Exception
  {
    Pose3d nearest = null;
    JsonNode root = mapper.readTree(json);
    JsonNode tags = root.get("Results").get("Fiducial");
    for (int i=0; i<tags.size(); ++i)
    {
      JsonNode tag = tags.get(i);
      int id = tag.get("fID").intValue();
      JsonNode pose = tag.get(POSE_DATA);
      if (pose == null  ||  pose.size() != 6)
        throw new Exception(POSE_DATA + " does not contain expected elements");
      // Map from camera into robot coordinates
      double x =  pose.get(2).doubleValue();
      double y = -pose.get(0).doubleValue();
      double z = -pose.get(1).doubleValue();      
      double angle = -pose.get(4).doubleValue();
      System.out.format("Tag #%2d @ %.2f, %.2f, %.2f m, %.1f degrees\n",
                        id, x, y, z, angle);
      if (nearest == null   ||    x < nearest.getX())
        nearest = new Pose3d(x, y, z, new Rotation3d(0.0, 0.0, Math.toRadians(angle)));
    }

    return nearest;
  }

  /** @return Nearest Apriltag pose in robot coordinates
   *  @throws Exception on error
   */
  public Pose3d getNearestTag() throws Exception
  {
    return decode(getJSON());
  }

  /** Standalone demo with saved data */
  public static void main(String[] args) throws Exception
  {
    String json = Files.readString(Path.of(LimelightClient.class.getResource("json.txt").toURI()));
    System.out.println(json);

    Pose3d nearest = decode(json);
    System.out.format("--------> %.2f, %.2f, %.2f m, %.1f degrees\n",
                      nearest.getTranslation().getX(),
                      nearest.getTranslation().getY(),
                      nearest.getTranslation().getZ(),
                      Math.toDegrees(nearest.getRotation().getZ()));
  }
}
