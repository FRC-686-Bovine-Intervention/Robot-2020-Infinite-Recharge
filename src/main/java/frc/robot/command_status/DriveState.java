package frc.robot.command_status;

import frc.robot.lib.util.DataLogger;

import com.ctre.phoenix.motorcontrol.*;

/**
 * Drivetrain status structure, filled by DriveLoop.java
 */
public class DriveState
{
	// singleton class
	private static DriveState instance = null;
	public static DriveState getInstance() 
	{ 
		if (instance == null) {
			instance = new DriveState();
		}
		return instance;
	}
	
	// all member variables should be private to force other object to use the set/get access methods
	// which are synchronized to allow multi-thread synchronization

	private ControlMode talonControlMode = ControlMode.Disabled;
	private NeutralMode neutralMode;
	
	private double lDistanceInches, rDistanceInches;
	private double lSpeedInchesPerSec, rSpeedInchesPerSec;
	private double heading;
	
	private double lMotorCurrent, rMotorCurrent;
	private double lMotorStatus, rMotorStatus;
	private int lMotorPIDError, rMotorPIDError;
	
	public DriveState() {}
	
	public synchronized void setTalonControlMode(ControlMode val) { talonControlMode = val; }
	public synchronized ControlMode getTalonControlMode() { return talonControlMode; }
	
	public synchronized void setNeutralMode(NeutralMode val) { neutralMode = val; }
	public synchronized NeutralMode getNeutralMode() { return neutralMode; }
	
	public synchronized void setLeftDistanceInches(double val)  { lDistanceInches = val; }
	public synchronized void setRightDistanceInches(double val) { rDistanceInches = val; }

	public synchronized double getLeftDistanceInches()  { return lDistanceInches; }
	public synchronized double getRightDistanceInches() { return rDistanceInches; }

	public synchronized void setLeftSpeedInchesPerSec(double val)  { lSpeedInchesPerSec = val; }
	public synchronized void setRightSpeedInchesPerSec(double val) { rSpeedInchesPerSec = val; }
	
	public synchronized double getLeftSpeedInchesPerSec()  { return lSpeedInchesPerSec; }
	public synchronized double getRightSpeedInchesPerSec() { return rSpeedInchesPerSec; }

	public synchronized void setMotorCurrent(double lVal, double rVal) { lMotorCurrent = lVal; rMotorCurrent = rVal; }
	public synchronized void setMotorStatus(double lVal, double rVal) { lMotorStatus = lVal; rMotorStatus = rVal; }				// current settings, read back from Talon (may be different than commanded values)
	public synchronized void setMotorPIDError(int lVal, int rVal) { lMotorPIDError = lVal; rMotorPIDError = rVal; }
    
	public synchronized double getLeftMotorCurrent()  { return lMotorCurrent; }
	public synchronized double getRightMotorCurrent() { return rMotorCurrent; }

	public synchronized double getLeftMotorCtrl()  { return lMotorStatus; }
	public synchronized double getRightMotorCtrl() { return rMotorStatus; }

	public synchronized double getLeftMotorPIDError()  { return lMotorPIDError; }
	public synchronized double getRightMotorPIDError() { return rMotorPIDError; }

	public synchronized void setHeadingDeg(double val) { setHeading(val*Math.PI/180.0); }
    public synchronized void setHeading(double val) { heading = val; }

    public synchronized double getHeading() { return heading; };
    public synchronized double getHeadingDeg() { return heading*180.0/Math.PI; }
	
    

    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        	synchronized (DriveState.this)
        	{
	    		put("DriveState/TalonControlMode", talonControlMode.toString() );
	    		put("DriveState/neutralMode", (neutralMode == NeutralMode.Coast ? "Coast" : "Brake"));
	    		put("DriveState/lMotorCurrent", lMotorCurrent );
	    		put("DriveState/rMotorCurrent", rMotorCurrent );
	    		put("DriveState/lMotorStatus", lMotorStatus );
	    		put("DriveState/rMotorStatus", rMotorStatus );
	    		put("DriveState/lSpeed", lSpeedInchesPerSec );	// used by RaspberryPi set LED velocity display
	    		put("DriveState/rSpeed", rSpeedInchesPerSec );	// used by RaspberryPi set LED velocity display
	    		put("DriveState/lDistance", lDistanceInches );
	    		put("DriveState/rDistance", rDistanceInches );
	    		put("DriveState/lPIDError",  lMotorPIDError );
	    		put("DriveState/rPIDError", rMotorPIDError );
	    		put("DriveState/Heading", getHeadingDeg() );
        	}
        }
    };
    
    public DataLogger getLogger() { return logger; }


    
	
}
