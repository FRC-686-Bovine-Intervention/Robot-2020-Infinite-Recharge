package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;

/**
 * An abstract class for robot steering controls.
 */
public abstract class SteeringBase 
{    
    // constructor
    protected SteeringBase() 
    {
    }

    public abstract DriveCommand getDriveCommand();	// mapping from joystick controls to DriveCommand

    public abstract DataLogger getLogger();
}
