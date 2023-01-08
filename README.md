Team 2393 FRC Season 2023
=========================

FRC Software manual: https://docs.wpilib.org/en/latest/index.html
  * Browse through all
  * Note expecially "Changelog", "Known Issues"
 
Done:
  * Kickoff: January 7, 2023, https://efcms.engr.utk.edu/efp/first-kickoff/2023/welcome.php
  * Swervebot: Manual and basic auto
  * Camera: Detect April tags

Next steps:
  * Study game manual, https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system
  * Install 2023 WPILib
  * Prepare robot skeleton
    * Try new RoboRIO
    * Get 3rd party libraries for CTRE, Vec, ...
      * https://github.com/CrossTheRoadElec/Phoenix-Releases/releases
      * https://www.revrobotics.com/software
    * Update firmware of power distribution module, pneumatic hub etc.
  * Prepare 'test' robots
    * Manual control of all the motor types (Falcon, Spark, ??) we might have in the robot
      * https://www.andymark.com
      * https://wcproducts.com
      * https://store.ctr-electronics.com/software
      * https://docs.revrobotics.com 
      * https://www.chiefdelphi.com/t/2023-falcon-updates-and-availability
    * PID control of each motor type for position as well as speed
    * While actual robot is being designed, work with swervebot
    * Prepare test robot for each key component of new robot as design materializes
      * https://www.projectb.net.au/resources/robot-mechanisms
    * Vision: Learn about April tags
      * https://docs.wpilib.org/en/latest/docs/software/vision-processing/apriltag/apriltag-intro.html
      * https://docs.photonvision.org/en/latest/docs/getting-started/description.html
      * https://github.com/Tigerbotics7125/AprilTag16h5
      * https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code
      * https://www.chiefdelphi.com/t/list-of-apriltag-3d-positions
    * Auto
      * https://www.chiefdelphi.com/t/team-3476-introduces-autobuilder


Events, https://frc-events.firstinspires.org/2023/Events/EventList:
 * Week 1, March 1, 2023: [Searcy, AR](https://maps.google.com/maps?ll=35.249098,-91.726211&z=16&t=m&hl=en-US&gl=US&mapclient=embed&q=Searcy%2C%20AR%2072149)
 * Week 5, March 29, 2023: [Knoxville, TN](https://maps.google.com/maps?ll=35.971789,-83.900286&z=13&t=m&hl=en-US&gl=US&mapclient=embed&q=Knoxville%2C%20TN%2037915)  


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

