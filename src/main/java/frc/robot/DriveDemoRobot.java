package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveDemoRobot extends CommandBaseRobot
{
    XboxController joystick = new XboxController(0);

    final int TICKS_PER_METER = 41800;

    WPI_TalonFX motor = new WPI_TalonFX(1);

    @Override
    public void robotInit()
    {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Coast);
    }

    /** @return Position in meters */
    double getPosition()
    {
        return motor.getSelectedSensorPosition() / TICKS_PER_METER;
    }

    @Override
    public void teleopPeriodic()
    {
        motor.setVoltage(joystick.getLeftY() * -10);

        SmartDashboard.putNumber("Ticks", motor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Position", getPosition());
    }
}
