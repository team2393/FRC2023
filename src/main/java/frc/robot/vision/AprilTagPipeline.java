// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.vision.VisionPipeline;

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

    // TODO Set up Pose Estimator
    // AprilTagPoseEstimator.Config poseEstConfig = new AprilTagPoseEstimator.Config(...);
    // AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);
  }

  @Override
  public void process(Mat image)
  {
      // Update stats
      ++image_count;
      long now = System.currentTimeMillis();
      boolean snapshot = now > start + 1000;
      if (snapshot)
      {
        double recent_fps = image_count * 1000.0 / (now - start);
        fps = 0.5*fps + 0.5*recent_fps;
        image_count = 0;
        start = now;
      }
        
      // Detector needs grayscale image
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
        
        // Is p1, p2, p3, p4 always left bottom, right bottom,
        // right top, left top?
        Point p1 = new Point(tag.getCornerX(0), tag.getCornerY(0));
        Point p2 = new Point(tag.getCornerX(1), tag.getCornerY(1));
        Point p3 = new Point(tag.getCornerX(2), tag.getCornerY(2));
        Point p4 = new Point(tag.getCornerX(3), tag.getCornerY(3));
        Imgproc.line(image, p1, p2, mark, 1);
        Imgproc.line(image, p2, p3, mark, 1);
        Imgproc.line(image, p3, p4, mark, 1);
        Imgproc.line(image, p4, p1, mark, 1);

        // Pose estimator?
        //    Transform3d pose = poseEstimator.estimate(tag);
        
        String info = String.format("%d", tag.getId());
        Imgproc.putText(image, info,
                        p4,
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, black, 3);
        Imgproc.putText(image, info,
                        p4,
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, white, 1);
        if (snapshot)
          System.out.format("ID %02d, Margin %4.1f\n",
                            tag.getId(),
                            tag.getDecisionMargin());
      }

      String info = String.format("%3.1f f/s", fps);
      Imgproc.putText(image, info, new Point(0, height-11), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, black, 2);
      Imgproc.putText(image, info, new Point(1, height-12), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, white, 1);
      // Give the output stream a new image to display
      output.putFrame(image);
    }
}
