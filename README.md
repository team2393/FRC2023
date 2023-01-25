Team 2393 FRC Season 2023
=========================

FRC Software manual: https://docs.wpilib.org/en/stable/index.html

Game manual: https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system

Done:
  * Kickoff: January 7, 2023, https://efcms.engr.utk.edu/efp/first-kickoff/2023/welcome.php
  * Swervebot: Manual and basic auto
  * Camera: Basic April tags demo
      * https://docs.wpilib.org/en/latest/docs/software/vision-processing/apriltag/apriltag-intro.html
      * https://docs.photonvision.org/en/latest/docs/getting-started/description.html
      * https://github.com/Tigerbotics7125/AprilTag16h5
      * https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code
      * https://www.chiefdelphi.com/t/list-of-apriltag-3d-positions
  * Install 2023 WPILib on robot laptop
    * Settings: Editor Tab Size 2, Detect indentation off, minimap disabled, inlay hints off
    
  * Get 3rd party libraries for CTRE, Vec, ...
    * https://github.com/CrossTheRoadElec/Phoenix-Releases/releases
    * https://www.revrobotics.com/software
   * Try new RoboRIO2

Next:
  * Update firmware of power distribution module, pneumatic hub etc.
  * Manual control of all the motor types (Falcon, Spark, ??) we might have in the robot
      * https://www.andymark.com
      * https://wcproducts.com
      * https://store.ctr-electronics.com/software
      * https://docs.revrobotics.com 
      * https://www.chiefdelphi.com/t/2023-falcon-updates-and-availability
  * PID control of each motor type for position as well as speed
  * While actual robot is being designed, work with swervebot
  * Vision
      * Calibrate camera
      * Demo of computing robot location on field
      * Use https://github.com/wpilibsuite/WPILibPi/releases ?
      * Use LimeLight, https://www.chiefdelphi.com/t/limelight-v2-2023-firmware/422597 ?
  * Prepare test robot for each key component of new robot as design materializes
     * https://www.projectb.net.au/resources/robot-mechanisms
  * Calibrate camera to get April tag locations
  * .. much more


Events, https://frc-events.firstinspires.org/2023/Events/EventList:
 * Week 1, March 1, 2023: [Searcy, AR](https://maps.google.com/maps?ll=35.249098,-91.726211&z=16&t=m&hl=en-US&gl=US&mapclient=embed&q=Searcy%2C%20AR%2072149)
 * Week 5, March 29, 2023: [Knoxville, TN](https://maps.google.com/maps?ll=35.971789,-83.900286&z=13&t=m&hl=en-US&gl=US&mapclient=embed&q=Knoxville%2C%20TN%2037915)  


RoboRIO2
--------

WPILib documentation describes how to flash the micro-SD card on a laptop.
Next, connect the RoboRIO via USB and use the roboRIO Imaging Tool go to Edit Startup Settings, fill out the Team Number box and hit Apply.
This failed on the first attempt, and simply worked the second time around.

Profiling
---------

'VisualVM', available from https://visualvm.github.io,
allows you to see how much CPU and memory the code is using on the RoboRIO,
and where it spends its time.

Assuming you unpacked it to \Users\Public\wpilib,
start it from a command prompt to pass the JDK location like this:

```
cd \Users\Public\wpilib\visualvm\bin
visualvm --jdkhome \Users\Public\wpilib\2022\jdk
```

In `build.gradle`, this addition to the FRCJavaArtifact section
allows VisualVM to access the JVM running on the robot:

```
frcJava(getArtifactTypeClass('FRCJavaArtifact')) {
    // Enable VisualVM connection
    jvmArgs.add("-Dcom.sun.management.jmxremote=true")
    jvmArgs.add("-Dcom.sun.management.jmxremote.port=1198")
    jvmArgs.add("-Dcom.sun.management.jmxremote.local.only=false")
    jvmArgs.add("-Dcom.sun.management.jmxremote.ssl=false")
    jvmArgs.add("-Dcom.sun.management.jmxremote.authenticate=false")
    jvmArgs.add("-Djava.rmi.server.hostname=10.23.93.2")     
}
```

To connect to the program running on the robot:
 * File, Add JMX Connection
 * 'Connection:' 172.22.11.2:1198 respectively 10.23.93.2:1198
 * Check 'Do not require SSL connection'
 * A new entry with a 'pid' should appear under the 'Remote' list.
   Double-click, then check 'Monitor', 'Sample.. CPU' etc.



Camera Calibration
------------------

The `AprilTagPoseEstimator` needs "focal lengths" and "center" values.

To get them via PhotonVision:

 * Get PhotonVision for the PC from https://github.com/PhotonVision/photonvision/releases
 * Plug USB camera into laptop
 * Run in cmd window (using JDK 11 from 2022, not 2023 wpilib!):
   `\Users\Public\wpilib\2022\jdk\bin\java -jar Downloads\photonvision-v2023.1.2-winx64.jar`
 * Open web browser to http://localhost:5800
 * Settings: Configure team number
 * Cameras: Select camera, download calibration target, ...
   See https://docs.photonvision.org/en/latest/docs/getting-started/pipeline-tuning/calibration.html
 * Settings: Export Settings, then find the camera config.json,
   look for calibration values,
   compare with https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/23



Limelight
---------

Initially, access as http://limelight.local:5801/

"Front" Camera:
Set team 2393.
Set IP address to static, 10.23.93.11, allowing access as http://10.23.93.11:5801/, and change name to "limelight-front".

Example pipelines are in limelight folder:
Pipeline 0: Drive
Pipeline 1: April (and make that the default)

Network table "limelight-front", "json": "t6t_rs" contains the x, y, z position

{"Results":{"Classifier":[],"Detector":[],"Fiducial":[{"fID":6,"fam":"16H5C","pts":[],"skew":[],"t6c_ts":[-0.1819821297174766,0.19152947927501754,-1.2927626167319477,9.536598317113642,5.656188787485489,0.8302258066244446],"t6r_fs":[-5.950337624721007,0.2342261550517298,0.2711905207249825,0.837496139143989,-9.45377348720073,174.20548791505757],"t6r_ts":[-0.1819821297174766,0.19152947927501754,-1.2927626167319477,9.536598317113642,5.656188787485489,0.8302258066244446],"t6t_cs":[0.050902202415996876,0.024601799378643218,1.3182715719623306,-9.501442252719597,-5.715398671275721,0.11734407372542792],"t6t_rs":[0.050902202415996876,0.024601799378643218,1.3182715719623306,-9.501442252719597,-5.715398671275721,0.11734407372542792],"ta":0.011568170972168446,"tx":2.081610679626465,"txp":169.65443420410156,"ty":-1.1051111221313477,"typ":124.498291015625}],"Retro":[],"botpose":[-5.950337201950147,0.2342294161416599,0.27119060041452736,0.8374914064004849,-9.453768731433065,174.20563327242846],"pID":1.0,"tl":24.398488998413086,"ts":512294.35746,"v":1}}