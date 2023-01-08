package frc.robot.drivetrain.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.vision.VisionThread;
import frc.robot.CommandBaseRobot;

public class CameraTestRobot extends CommandBaseRobot
{
  @Override
  public void robotInit()
  {
    super.robotInit();
  
    final int width = 640, height = 480;

    // TODO VisionThread
    Thread m_visionThread = new Thread(() ->
    {
      // Get and configure camera
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(width, height);
      
      // Setup a CvSource. This will send images back to the Dashboard
      CvSource output = CameraServer.putVideo("Processed", width, height);
      
      final Scalar black = new Scalar(0, 0, 0);
      final Scalar white = new Scalar(255, 255, 255);
      
      // Mats are very memory expensive. Lets reuse this Mat.
      Mat orig = new Mat();
      Mat gray = new Mat();
      
      // Get a CvSink. This will capture Mats from the camera
      CvSink source = CameraServer.getVideo();

      AprilTagDetector detector = new AprilTagDetector();
      AprilTagDetector.Config config = new AprilTagDetector.Config();
      detector.setConfig(config);
      detector.addFamily("tag16h5");

      int image_count = 0;
      long start = System.currentTimeMillis();
      double fps = 0.0;

      // Run until restarting or deploying robot code
      while (!Thread.interrupted())
      { // Grab a frame
        if (source.grabFrame(orig) == 0)
        {
          // Send the output the error.
          output.notifyError(source.getError());
          // skip the rest of the current iteration
          continue;
        }
        ++image_count;

        Imgproc.cvtColor(orig, gray, Imgproc.COLOR_BGR2GRAY);

        // // Put a rectangle on the image
        // Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), white, 5);

        // TODO Create 8 bit image, then detect tags
        // AprilTagDetection[] tags = detector.detect(mat);
        // for (AprilTagDetection tag : tags)
        // {
        //   Point p1 = new Point(tag.getCornerX(0), tag.getCornerY(0));
        //   Point p2 = new Point(tag.getCornerX(1), tag.getCornerY(1));
        //   Point p3 = new Point(tag.getCornerX(2), tag.getCornerY(2));
        //   Point p4 = new Point(tag.getCornerX(3), tag.getCornerY(3));
        //   Imgproc.line(mat, p1, p2, white, 4);
        //   Imgproc.line(mat, p2, p3, white, 4);
        //   Imgproc.line(mat, p3, p4, white, 4);
        //   Imgproc.line(mat, p4, p1, white, 4);
        // }

        long now = System.currentTimeMillis();
        if (now > start + 1000)
        {
          double recent_fps = image_count * 1000.0 / (now - start);
          fps = 0.5*fps + 0.5*recent_fps;
          image_count = 0;
          start = now;
        }
        String info = String.format("%.1f FPS", fps);
        Imgproc.putText(gray, info, new Point(0, height-15), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, black, 2);
        Imgproc.putText(gray, info, new Point(1, height-16), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, white, 1);
        // Give the output stream a new image to display
        output.putFrame(gray);
      }
    });

    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }
}
