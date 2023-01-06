// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.parts.RotationEncoder;
import frc.robot.parts.SparkMini;

/** Rotational part of a swerve module */
public class Rotator 
{
    private int channel;
    private SparkMini motor;
    private RotationEncoder encoder;

    /** Construct rotator
     *  @param channel PMW channel 0-4
     *  @param offset Offset from 'forward' in degrees
     */
    public Rotator (int channel, double offset)
    {
        this.channel = channel;
        motor = new SparkMini(channel);
        encoder = new RotationEncoder(channel, offset);

        SmartDashboard.setDefaultNumber("Offset" + channel, offset);
        SmartDashboard.setDefaultNumber("P", 0.05);
    }

    /** @param speed Speed -1..1 for rotating the swerve module */
    public void run(double speed)
    {
        motor.set(speed);
        SmartDashboard.putNumber("Angle" + channel, encoder.getHeading().getDegrees());
    }

    /** @param desired Desired angle of serve module in degrees */
    public void setAngle(double desired)
    {
        encoder.setZero(SmartDashboard.getNumber("Offset"+channel, 0));
        // Proportional control, with error normalized to -180..180
        double angle = encoder.getHeading().getDegrees();
        double error = Math.IEEEremainder(desired - angle, 360.0);
        double output = error*SmartDashboard.getNumber("P", 0);
        SmartDashboard.putNumber("Angle" + channel, encoder.getHeading().getDegrees());
        SmartDashboard.putNumber("Desired" + channel, desired);
        motor.set(output);
    }

    /** @return Angle in degrees */
    public Rotation2d getAngle()
    {
        return encoder.getHeading();
    }
}
