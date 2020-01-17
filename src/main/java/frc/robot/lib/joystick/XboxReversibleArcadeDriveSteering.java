package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.joystick.SteeringLib.ThrottleTurn;
import frc.robot.lib.util.DataLogger;

/**
 * Implements a simple arcade drive, where single stick is used for throttle and turn.
 */
public class XboxReversibleArcadeDriveSteering extends ReversibleSteeringBase
{
    JoystickBase controls;
 
    DeadbandNonLinearity deadbandNonLinearity;

	double rThrottle;
	double rTurn;
	double lThrottle;
	double lTurn;
    ThrottleTurn throttleTurn = new ThrottleTurn();
    DriveCommand driveCmd = new DriveCommand(0,0); 

    boolean usingLeftStick = false;     // default this way so autonomous uses hatch camera
	double kCrossoverThreshold = 0.2;
    boolean leftStickActive = false;
    boolean rightStickActive = false;

    public XboxReversibleArcadeDriveSteering(JoystickBase _controls, DeadbandNonLinearity _deadbandNonLinearity) 
    {
        controls = _controls;   // port selected in DriverControls
        deadbandNonLinearity = _deadbandNonLinearity;
    }

    // get left and right motor speeds
	public DriveCommand getDriveCommand()
	{      
		throttleTurn = getThrottleTurn();
		driveCmd = SteeringLib.arcadeDrive(throttleTurn.throttle, throttleTurn.turn, deadbandNonLinearity);
		return driveCmd; 
	}       


    // get throttle and turn parameters from joysticks
    public ThrottleTurn getThrottleTurn()
    {
		rThrottle = -controls.getAxis(Xbox.kRStickYAxis);
		rTurn     = -controls.getAxis(Xbox.kRStickXAxis);
		lThrottle = -controls.getAxis(Xbox.kLStickYAxis);
		lTurn     = -controls.getAxis(Xbox.kLStickXAxis);
        
        ThrottleTurn out = new ThrottleTurn();
		out.throttle = lThrottle;
		out.turn = lTurn;

        leftStickActive =  ((Math.abs(lThrottle) >= kCrossoverThreshold) || (Math.abs(lTurn) >= kCrossoverThreshold));
        rightStickActive = ((Math.abs(rThrottle) >= kCrossoverThreshold) || (Math.abs(rTurn) >= kCrossoverThreshold));

		// check to see if we are switching sticks
		if (usingLeftStick)
		{
			if (rightStickActive && !leftStickActive) {
				usingLeftStick = false;
			}
		}
		else 
		{
			if (leftStickActive && !rightStickActive) {
				usingLeftStick = true;
			}
		}
	
		// if we are driving in reverse, flip stick controls
		if (!usingLeftStick)
		{
			out.throttle = -rThrottle;
			out.turn = +rTurn;
        }       
        
// System.out.printf("lThrottle: %5.3f, lTurn: %5.3f, rThrottle: %5.3f, rTurn: %5.3f, lAct:%b, rAct:%b, usingLeft:%b\n", lThrottle, lTurn, rThrottle, rTurn, leftStickActive, rightStickActive, usingLeftStick);

        return out;
    }

	public boolean usingLeftStick()
	{
		return usingLeftStick;
	}

    public boolean joystickActive()
    {
		// in autonomous, we want to check if driver is attempting to override autonomous
		// by moving joystick.  This is the function that does this.
		
		getThrottleTurn();

        return leftStickActive || rightStickActive;
    }
    



    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("XboxReversibleArcadeDriveSteering/throttle", throttleTurn.throttle);
            put("XboxReversibleArcadeDriveSteering/turn", throttleTurn.turn);
            put("XboxReversibleArcadeDriveSteering/left", driveCmd.getLeftMotor());
            put("XboxReversibleArcadeDriveSteering/right", driveCmd.getRightMotor());
        }
    };        
}

