package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.PTOTansmissionState;

/**
 * Implements a simple arcade drive, where single stick is used for throttle and
 * turn.
 */
public class TmArcadeDriveSteering extends SteeringBase
{
    JoystickBase stick;
    DeadbandNonLinearity deadbandNonLinearity;
	double throttle;
	double turn;
    DriveCommand driveCmd = new DriveCommand(0,0); 

    public TmArcadeDriveSteering(JoystickBase _stick, DeadbandNonLinearity _deadbandNonLinearity) 
    {
        stick = _stick;   // port selected in DriverControls
        deadbandNonLinearity = _deadbandNonLinearity;
    }

    // get left and right motor speeds
	public DriveCommand getDriveCommand()
	{      
        throttle = -stick.getAxis(Thrustmaster.kYAxis); 
        turn =     -stick.getAxis(Thrustmaster.kXAxis);
        
        //'Turning' is reversed for lift
        if(Lift.getInstance().getPTOState() == PTOTansmissionState.LIFT_ENABLED){
            driveCmd = SteeringLib.arcadeDrive(throttle, -turn, deadbandNonLinearity);
        } else {
            driveCmd = SteeringLib.arcadeDrive(throttle, turn, deadbandNonLinearity);
        }
		return driveCmd; 
    }   
    
    


    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("TmArcadeDriveSteering/throttle", throttle);
            put("TmArcadeDriveSteering/turn", turn);
            put("TmArcadeDriveSteering/left", driveCmd.getLeftMotor());
            put("TmArcadeDriveSteering/right", driveCmd.getRightMotor());
        }
    };            
}
