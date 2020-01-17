package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.util.DataLogger;


/**
 * Implements a simple arcade drive, where single stick is used for throttle and
 * turn.
 */
public class XboxTankDriveSteering extends SteeringBase
{
    JoystickBase controls;
    DeadbandNonLinearity deadbandNonLinearity;
    double lSpeed;
    double rSpeed;
    DriveCommand driveCmd = new DriveCommand(0,0); 

    public XboxTankDriveSteering(JoystickBase _controls, DeadbandNonLinearity _deadbandNonLinearity) 
    {
        controls = _controls;   // port selected in DriverControlsXbox
        deadbandNonLinearity = _deadbandNonLinearity;
    }

    // get left and right motor speeds
	public DriveCommand getDriveCommand()
	{      
        lSpeed = -controls.getAxis(Xbox.kLStickYAxis); 
        rSpeed = -controls.getAxis(Xbox.kRStickYAxis);
        driveCmd = SteeringLib.tankDrive(lSpeed, rSpeed, deadbandNonLinearity);
		return driveCmd; 
	}       
   
    

    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("XboxTankDriveSteering/lSpeed", lSpeed);
            put("XboxTankDriveSteering/rSpeed", rSpeed);
            put("XboxTankDriveSteering/left", driveCmd.getLeftMotor());
            put("XboxTankDriveSteering/right", driveCmd.getRightMotor());
        }
    };  
}

