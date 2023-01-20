package frc.robot.tutorial;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CommandBaseRobot;

public class FirstDemoRobot extends CommandBaseRobot
{
    private XboxController joystick = new XboxController(0);

    @Override
    public void autonomousPeriodic()
    {
        System.out.println("Doing something on my own...");
    }

    @Override
    public void teleopInit()
    {
        System.out.println("Entering tele-op...");
    }    

    @Override
    public void teleopPeriodic()
    {
        if (joystick.getAButtonPressed())
            System.out.println("You pushed A");

        if (joystick.getBButton())
            System.out.println("You're holding B");

        if (RobotController.getUserButton())
            System.out.println("PRESSED!");
        
        System.out.println("Right X = " + joystick.getRightX());
    }
}
