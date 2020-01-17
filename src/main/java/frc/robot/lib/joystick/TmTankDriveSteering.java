package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.util.DataLogger;

/**
 * Implements a simple tank drive
 */
public class TmTankDriveSteering extends SteeringBase
{
    JoystickBase lStick;
    JoystickBase rStick;
    DeadbandNonLinearity deadbandNonLinearity;
    double lSpeed;
    double rSpeed;
    DriveCommand driveCmd = new DriveCommand(0,0); 

    public TmTankDriveSteering(JoystickBase _lStick, JoystickBase _rStick, DeadbandNonLinearity _deadbandNonLinearity) 
    {
        lStick = _lStick;   // port selected in DriverControls
        rStick = _rStick;
        deadbandNonLinearity = _deadbandNonLinearity;
    }

    // get left and right motor speeds
	public DriveCommand getDriveCommand()
	{      
        lSpeed = -lStick.getAxis(Thrustmaster.kYAxis); 
        rSpeed = -rStick.getAxis(Thrustmaster.kYAxis);
		driveCmd = SteeringLib.tankDrive(lSpeed, rSpeed, deadbandNonLinearity);
		return driveCmd; 
    }  
    
    

    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("TmTankDriveSteering/lSpeed", lSpeed);
            put("TmTankDriveSteering/rSpeed", rSpeed);
            put("TmTankDriveSteering/left", driveCmd.getLeftMotor());
            put("TmTankDriveSteering/right", driveCmd.getRightMotor());
        }
    };                
}
