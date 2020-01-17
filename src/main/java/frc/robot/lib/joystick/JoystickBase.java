package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.lib.util.DataLogger;

public abstract class JoystickBase extends Joystick
{
    // constructor
    public JoystickBase(int _port)
    {
        super(_port);
    }

    public boolean getButton(int _button)
    {
        return getRawButton(_button);
    }

    public boolean getButtonPressed(int _button)
    {
        return getRawButtonPressed(_button);
    }

    public boolean getButtonReleased(int _button)
    {
        return getRawButtonReleased(_button);
    }

    public double getAxis(int _axis)
    {
        return getRawAxis(_axis);
    }

    public boolean getAxisAsButton(int _axis)
    {
        return(getRawAxis(_axis) > 0.5);
    }

    // public int getPOV() already exists in Joystick class


    public abstract DataLogger getLogger();
}