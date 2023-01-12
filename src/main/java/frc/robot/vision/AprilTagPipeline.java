// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.Objects;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Vision pipeline that detects April tags */
public class AprilTagPipeline implements VisionPipeline
{  
  final private int width, height;

  // Output for processed image
  final private CvSource output;
  
  // Colors to use in output
  final private Scalar black = new Scalar(0, 0, 0);
  final private Scalar white = new Scalar(255, 255, 255);
  final private Scalar mark = new Scalar(255, 100, 255);
  
  // Re-used Mat for grayscale
  final private Mat gray;

  final private AprilTagDetector detector;
  final private AprilTagPoseEstimator estimator;

  // Stats
  int image_count = 0;
  long start = System.currentTimeMillis();
  double fps = 0.0;

  public AprilTagPipeline(int width, int height)
  {
    this.width = width;
    this.height = height;

    output = CameraServer.putVideo("Processed", width, height);

    gray = new Mat();

    // Tag detector
    detector = new AprilTagDetector();
    AprilTagDetector.Config config = new AprilTagDetector.Config();
    detector.setConfig(config);
    detector.addFamily("tag16h5");

    // Set up Pose Estimator
    // TODO Configure focal lengths
    // Compare PhotonVision calibration at 
    // "resolution" : { "width" : 320.0, "height" : 240.0 },
    // "cameraIntrinsics" : {
    //   "rows" : 3,
    //   "cols" : 3,
    //   "type" : 6,
    //   "data" : [ 302.9,   0.0, 153.3,
    //                0.0, 303.1, 138.3,
    //                0.0,   0.0,   1.0 ]
    AprilTagPoseEstimator.Config pose_config = new AprilTagPoseEstimator.Config(
        0.144,   // tag size in meters (14.4 cm. Actual field uses 15.24cm?)
        320,     // camera horizontal focal length, in pixels
        320,     // camera vertical focal length, in pixels
        width/2, // camera horizontal focal center, in pixels
        height/2  //camera vertical focal center, in pixels
    );
    SmartDashboard.setDefaultNumber("CameraF",  pose_config.fx);
    SmartDashboard.setDefaultNumber("CameraCX", pose_config.cx);
    SmartDashboard.setDefaultNumber("CameraCY", pose_config.cy);

    estimator = new AprilTagPoseEstimator(pose_config);
  }

  // TODO Demo of converting april tag poses to robot/field cooords
  // Use rotation?
  public static void main(String[] args)
  {
    // Apriltag pose coordinates:
    // Z = distance from camera,
    // X = "right",
    // Y = "down"
    Translation3d tag = new Translation3d(-2, -3, 1);
    System.out.println(tag);

    // Robot coordinates:
    // X = "forward"
    // Y = "left"
    // Z = "up"
    Translation3d fieldpos = new Translation3d(tag.getZ(), -tag.getX(), -tag.getY());
    // Expect 1, 2, 3
    System.out.println(fieldpos);
  }

  @Override
  public void process(Mat image)
  {
      // Update stats
      ++image_count;
      // Take snapshot every ~ second
      long now = System.currentTimeMillis();
      boolean snapshot = now > start + 1000;
      if (snapshot)
      { // Frames per second for last ~1000 ms
        double recent_fps = image_count * 1000.0 / (now - start);
        // Smoothing: 50% of last fps, 50% of this reading
        fps = 0.5*fps + 0.5*recent_fps;
        // Reset for next snapshot
        image_count = 0;
        start = now;
      }
        
      // AprilTagDetection needs grayscale image
      Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
      AprilTagDetection[] tags = detector.detect(gray);
      for (AprilTagDetection tag : tags)
      {
        // Ignore tags with hamming error correction
        if (tag.getHamming() > 0)
          continue;
        // In first tests, "good" tags had a margin of 80 to 100,
        // "bad" tags less than 10
        if (tag.getDecisionMargin() < 50.0f)
          continue;
        
        // left bottom, right bottom, right top, left top?
        Point p1 = new Point(tag.getCornerX(0), tag.getCornerY(0));
        Point p2 = new Point(tag.getCornerX(1), tag.getCornerY(1));
        Point p3 = new Point(tag.getCornerX(2), tag.getCornerY(2));
        Point p4 = new Point(tag.getCornerX(3), tag.getCornerY(3));
        Imgproc.line(image, p1, p2, mark, 1);
        Imgproc.line(image, p2, p3, mark, 1);
        Imgproc.line(image, p3, p4, mark, 1);
        Imgproc.line(image, p4, p1, mark, 1);

        // Pose estimator
        var cfg = estimator.getConfig();
        // TODO Pose coordinates:
        // Z = distance from camera,
        // X = "right",
        // Y = "down"?
        // First adjust fx and fy to get correct "Z", distance from camera?
        cfg.fx = cfg.fy = SmartDashboard.getNumber("CameraF", 320);
        // Are cx and cy the zero for pose "X" and "Y",
        // can be set to center of image?
        cfg.cx = SmartDashboard.getNumber("CameraCX", width/2);
        cfg.cy = SmartDashboard.getNumber("CameraCY", height/2);
        estimator.setConfig(cfg);
        Transform3d pose = estimator.estimate(tag);
        // TODO Lookup tag's position, then transform pose into absolute field position of robot

        String info = String.format("%d", tag.getId());
        Imgproc.putText(image, info,
                        p4,
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, black, 3);
        Imgproc.putText(image, info,
                        p4,
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, white, 1);
        if (snapshot)
          System.out.format("ID %02d, Margin %4.1f: %s\n",
                            tag.getId(),
                            tag.getDecisionMargin(),
                            Objects.toString(pose.getTranslation()));
      }

      String info = String.format("%3.1f f/s", fps);
      Imgproc.putText(image, info, new Point(0, height-11), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, black, 2);
      Imgproc.putText(image, info, new Point(1, height-12), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, white, 1);
      // Give the output stream a new image to display
      output.putFrame(image);
    }
}
