package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.util.DataLogger;


/**
 * Implements a simple arcade drive, where single stick is used for throttle and
 * turn.
 */
public class XboxPolarArcadeDriveSteering extends SteeringBase
{
    JoystickBase controls;
    DeadbandNonLinearity deadbandNonLinearity;
	double throttle;
	double turn;
    DriveCommand driveCmd = new DriveCommand(0,0); 

    public XboxPolarArcadeDriveSteering(JoystickBase _controls, DeadbandNonLinearity _deadbandNonLinearity) 
    {
        controls = _controls;   // port selected in DriverControlsXbox
        deadbandNonLinearity = _deadbandNonLinearity;
    }

    // get left and right motor speeds
	public DriveCommand getDriveCommand()
	{      
        double x = -controls.getAxis(Xbox.kLStickXAxis);
        double y = -controls.getAxis(Xbox.kLStickYAxis); 

	    turn = 0;
	    throttle = 0;
	    
    	if (-y > 0)
    	{
    		double theta = Math.atan(x/y);
    		throttle = Math.sqrt(x*x + y*y);
    		turn = theta*2/Math.PI;
    	}
    	else if (-y < 0)
    	{
    		double theta = Math.atan(-x/y);
    		throttle = -Math.sqrt(x*x + y*y);
    		turn = theta*2/Math.PI;
    	}
    	else
    	{
    		throttle = Math.abs(x);
			turn = x;
    	}

        driveCmd = SteeringLib.arcadeDrive(throttle, turn, deadbandNonLinearity);
		return driveCmd; 
    }    
    
    


    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("XboxPolarArcadeDriveSteering/throttle", throttle);
            put("XboxPolarArcadeDriveSteering/turn", turn);
            put("XboxPolarArcadeDriveSteering/left", driveCmd.getLeftMotor());
            put("XboxPolarArcadeDriveSteering/right", driveCmd.getRightMotor());
        }
    };            
}

