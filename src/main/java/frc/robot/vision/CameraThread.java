// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.vision.VisionPipeline;

/** Thread that reads from camera and calls {@link VisionPipeline}
 * 
 *  Separates the task of handling the camera
 *  from the processing of those images.
 * 
 *  Passing the pipeline to use into this class
 *  means that the pipeline needs to be constructed first,
 *  but some CV code simply doesn't work until it's initialized
 *  by calling the CameraServer.
 * 
 *  So this class takes a Supplier<VisionPipeline>, which means
 *  it can first create the camera and thus initialize CV,
 *  then create the vision pipeline via the Supplier.
 */
public class CameraThread extends Thread
{
  final int width, height;
  final Supplier<VisionPipeline> pipeline_getter;

  public CameraThread(int width, int height, Supplier<VisionPipeline> pipeline_getter)
  {
    this.width = width;
    this.height = height;
    this.pipeline_getter = pipeline_getter;
    setName("Vision");
    setDaemon(true);
  }

  @Override
  public void run()
  {
    // Get and configure camera
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(width, height);
    CvSink source = CameraServer.getVideo();
      
    // Re-used Mat for image
    Mat image = new Mat();

    // Create the pipeline
    VisionPipeline pipeline = pipeline_getter.get();
      
    // Run until restarting or deploying robot code
    while (!Thread.interrupted())
      if (source.grabFrame(image) == 0)
        System.err.println(source.getError());
      else
        pipeline.process(image);
  }
}
