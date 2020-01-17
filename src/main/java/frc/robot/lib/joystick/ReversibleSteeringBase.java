package frc.robot.lib.joystick;

/**
 * An abstract class for steering controls where the robot's "forward" direction can be reveresed by switching joysticks
 */
public abstract class ReversibleSteeringBase extends SteeringBase
{    
    // constructor
    protected ReversibleSteeringBase() 
    {
    }

    public abstract boolean usingLeftStick();
    public abstract boolean joystickActive();

}
