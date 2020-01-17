package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.util.DataLogger;


/**
 * Implements a simple arcade drive, where single stick is used for throttle and
 * turn.
 */
public class XboxTriggerDriveSteering extends SteeringBase
{
    JoystickBase controls;
    DeadbandNonLinearity deadbandNonLinearity;
    boolean quickTurn = false;
    double throttle; 
    double turn;
    DriveCommand driveCmd = new DriveCommand(0,0); 

    public XboxTriggerDriveSteering(JoystickBase _controls, DeadbandNonLinearity _deadbandNonLinearity) 
    {
        controls = _controls;   // port selected in DriverControlsXbox
        deadbandNonLinearity = _deadbandNonLinearity;
    }

    // store quickTurn
	public DriveCommand getDriveCommand(boolean _quickTurn)
	{     
        quickTurn = _quickTurn; 
		return getDriveCommand(); 
    }       
    
    // get left and right motor speeds
	public DriveCommand getDriveCommand()
	{      
    	throttle = controls.getAxis(Xbox.kRTriggerAxis) - controls.getAxis(Xbox.kLTriggerAxis);
        turn     = controls.getAxis(Xbox.kLStickXAxis);
        driveCmd = SteeringLib.triggerDrive(throttle, turn, deadbandNonLinearity, quickTurn);
		return driveCmd; 
	}       



    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("XboxTriggerDriveSteering/throttle", throttle);
            put("XboxTriggerDriveSteering/turn", turn);
            put("XboxTriggerDriveSteering/left", driveCmd.getLeftMotor());
            put("XboxTriggerDriveSteering/right", driveCmd.getRightMotor());
        }
    };       
}

