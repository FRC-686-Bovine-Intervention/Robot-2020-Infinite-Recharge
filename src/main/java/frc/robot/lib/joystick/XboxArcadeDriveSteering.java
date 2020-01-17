package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.util.DataLogger;


/**
 * Implements a simple arcade drive, where single stick is used for throttle and
 * turn.
 */
public class XboxArcadeDriveSteering extends SteeringBase
{
    JoystickBase controls;
    DeadbandNonLinearity deadbandNonLinearity;
    double throttle; 
    double turn;
    DriveCommand driveCmd = new DriveCommand(0,0); 

    public XboxArcadeDriveSteering(JoystickBase _controls, DeadbandNonLinearity _deadbandNonLinearity) 
    {
        controls = _controls;   // port selected in DriverControlsXbox
        deadbandNonLinearity = _deadbandNonLinearity;
    }

    // get left and right motor speeds
	public DriveCommand getDriveCommand()
	{      
        throttle = -controls.getAxis(Xbox.kLStickYAxis); 
        turn =     -controls.getAxis(Xbox.kLStickXAxis);
        driveCmd = SteeringLib.arcadeDrive(throttle, turn, deadbandNonLinearity);
		return driveCmd; 
    }   
    
    
    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("XboxArcadeDriveSteering/throttle", throttle);
            put("XboxArcadeDriveSteering/turn", turn);
            put("XboxArcadeDriveSteering/left", driveCmd.getLeftMotor());
            put("XboxArcadeDriveSteering/right", driveCmd.getRightMotor());
        }
    };                
}

