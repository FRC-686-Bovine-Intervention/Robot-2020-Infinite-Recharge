package frc.robot.lib.joystick;



// for 2019 Deep Space, we use a reversible joystick scheme that has a few extra functions

public abstract class ReversibleDriverControlsBase extends DriverControlsBase
{
    public abstract boolean getDrivingCargo();
    public abstract boolean joystickActive();
}
