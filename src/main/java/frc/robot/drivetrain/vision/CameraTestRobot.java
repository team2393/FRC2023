package frc.robot.drivetrain.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.CommandBaseRobot;

public class CameraTestRobot extends CommandBaseRobot
{
  @Override
  public void robotInit()
  {
    super.robotInit();
    Scalar white = new Scalar(255, 255, 255);

    Thread m_visionThread = new Thread(() ->
    {
      // Get the UsbCamera from CameraServer
      UsbCamera camera = CameraServer.startAutomaticCapture();

      // Set the resolution
      camera.setResolution(640, 480);

      // Get a CvSink. This will capture Mats from the camera
      CvSink cvSink = CameraServer.getVideo();

      // Setup a CvSource. This will send images back to the Dashboard
      CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

      // Mats are very memory expensive. Lets reuse this Mat.
      Mat mat = new Mat();

      // This cannot be 'true'. The program will never exit if it is. This
      // lets the robot stop this thread when restarting robot code or
      // deploying.
      while (!Thread.interrupted())
      { // Grab a frame
        if (cvSink.grabFrame(mat) == 0)
        {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());

          // skip the rest of the current iteration
          continue;
        }

        // Put a rectangle on the image
        Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), white, 5);

        // Give the output stream a new image to display
        outputStream.putFrame(mat);
      }
  });

  m_visionThread.setDaemon(true);
  m_visionThread.start();
  }
}
