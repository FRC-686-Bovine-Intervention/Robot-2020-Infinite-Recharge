package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.util.DataLogger;

/**
 * Implements a simple tank drive
 */
public class TmTwoStickDriveSteering extends SteeringBase
{
    JoystickBase lStick;
    JoystickBase rStick;
    double throttle;
    double turn;
    DeadbandNonLinearity deadbandNonLinearity;
    DriveCommand driveCmd = new DriveCommand(0,0); 

    public TmTwoStickDriveSteering(JoystickBase _lStick, JoystickBase _rStick, DeadbandNonLinearity _deadbandNonLinearity) 
    {
        lStick = _lStick;   // port selected in DriverControls
        rStick = _rStick;
        deadbandNonLinearity = _deadbandNonLinearity;
    }

    // get left and right motor speeds
	public DriveCommand getDriveCommand()
	{      
        throttle = -lStick.getAxis(Thrustmaster.kYAxis);          //  left stick Y-axis for throttle
        turn =     -rStick.getAxis(Thrustmaster.kXAxis);          // right stick X-axis for turn
		driveCmd = SteeringLib.tankDrive(throttle, turn, deadbandNonLinearity);
		return driveCmd; 
    }    
    
    

    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("TmTwoStickDriveSteering/throttle", throttle);
            put("TmTwoStickDriveSteering/turn", turn);
            put("TmTwoStickDriveSteering/left", driveCmd.getLeftMotor());
            put("TmTwoStickDriveSteering/right", driveCmd.getRightMotor());
        }
    };    
}
