package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.util.DataLogger;


/**
 * Implements Team 254's Cheesy Drive. "Cheesy Drive" simply means that
 * the "turning" stick controls the curvature of the robot's path rather than
 * its rate of heading change. This helps make the robot more controllable at
 * high speeds. Also handles the robot's quick turn functionality - "quick turn"
 * overrides constant-curvature turning for turn-in-place maneuvers.
 */
public class XboxCheesyTwoStickDriveSteering extends SteeringBase
{
    JoystickBase controls;
    DeadbandNonLinearity deadbandNonLinearity;
    boolean quickTurn = false;
    double kTurnSensitivity;
    double throttle; 
    double turn;
    DriveCommand driveCmd = new DriveCommand(0,0); 

    public XboxCheesyTwoStickDriveSteering(JoystickBase _controls, DeadbandNonLinearity _deadbandNonLinearity, double _kTurnSensitivity) 
    {
        controls = _controls;   // port selected in DriverControlsXbox
        deadbandNonLinearity = _deadbandNonLinearity;
        kTurnSensitivity = _kTurnSensitivity;
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
        throttle = -controls.getAxis(Xbox.kLStickYAxis);     //  left stick Y-axis for throttle
        turn =     -controls.getAxis(Xbox.kRStickXAxis);     // right stick X-axis for turn
        driveCmd = SteeringLib.cheesyDrive(throttle, turn, deadbandNonLinearity, quickTurn, kTurnSensitivity);
		return driveCmd; 
    }   
    
    

    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("XboxCheesyTwoStickDriveSteering/throttle", throttle);
            put("XboxCheesyTwoStickDriveSteering/turn", turn);
            put("XboxCheesyTwoStickDriveSteering/left", driveCmd.getLeftMotor());
            put("XboxCheesyTwoStickDriveSteering/right", driveCmd.getRightMotor());
        }
    };             

}

