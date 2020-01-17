package frc.robot.command_status;



import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Kinematics.WheelSpeed;

import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.Timer;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.  
 * The command is set by Drive.java, and read by DriveLoop.java, which sends it to the drive motors
 */

/*
 * Drive.java and DriveLoop.java implement the five differential-drive control modes in the DriveControlMode enumeration
 * 
 * OPEN_LOOP:    		Used in Teleop.  Joystick controls are mapped into left/right motor speeds, using
 *               		the getDriveCommand() functions in lib.joystick.  Motor controllers are placed in %Vbus mode.
 *                
 * BASE_LOCKED:  		The robot attempts to maintain it's current position.  
 *               		Motor controllers use a position PID loop, and brake mode is enabled.
 *                                 
 * VELOCITY_SETPOINT:	The robot's left and right motors attempt to maintain the commanded velocity.  This should
 *                      result in travel with a constant curvature.  Motor controllers use velocity PID loops to
 *                      maintain velocity setpoints.                                
 *                                 
 * VELOCITY_HEADING:	Like VELOCITY_SETPOINT, with the addition that gyro feedback is used to adjust the left/right 
 * 						motor velocity setpoints in order to maintain a given heading.  Useful for traveling in a straight line.                                 
 *                      Motor controllers use velocity PID to maintain velocity setpoints, RoboRIO uses heading PID
 *                      to adjust velocity setpoints to maintain heading. 
 *                                
 */

public class DriveCommand
{    
	// The robot drivetrain's various states
	public enum DriveControlMode { OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING }

	// all member variables should be private to force other object to use the set/get access methods
	// which are synchronized to allow multi-thread synchronization	
	private DriveControlMode driveMode = DriveControlMode.OPEN_LOOP;
	private ControlMode talonMode = ControlMode.PercentOutput;
	private WheelSpeed wheelSpeed = new WheelSpeed();
	private static NeutralMode neutralMode;	// Brake or Coast
	private boolean resetEncoders;
    private double commandTime;
    
    public DriveCommand(double _left, double _right)
    {
        this(DriveControlMode.OPEN_LOOP, _left, _right, NeutralMode.Coast);
    }

    public DriveCommand(double _left, double _right, NeutralMode _neutralMode)
    {
        this(DriveControlMode.OPEN_LOOP, _left, _right, _neutralMode);
    }

    public DriveCommand(DriveControlMode _mode, double _left, double _right, NeutralMode _neutralMode) 
    {
    	setDriveMode(_mode);
    	setMotors(_left, _right);
    	setNeutralMode(_neutralMode);
    	resetEncoders = false;
    }

    public DriveCommand(DriveControlMode _mode, WheelSpeed _vWheel, NeutralMode _neutralMode) 
    {
    	setDriveMode(_mode);
    	setMotors(_vWheel);
    	setNeutralMode(_neutralMode);
    	resetEncoders = false;
    }

    public void scale(double _factor)
    {
    	wheelSpeed.scale(_factor);
    }
    
    
    public synchronized void setDriveMode(DriveControlMode _driveMode) 
    {
    	driveMode = _driveMode;
    	switch (driveMode)
    	{
    	case OPEN_LOOP:
    		talonMode = ControlMode.PercentOutput;
    		break;
    		
    	case BASE_LOCKED:
    		talonMode = ControlMode.Position;
    		
    	case VELOCITY_SETPOINT:
    	case VELOCITY_HEADING:
     		talonMode = ControlMode.Velocity;
    		break;
    		
    	default:
    		talonMode = ControlMode.Disabled;
    		break;
    	}
    }
    public synchronized DriveControlMode getDriveControlMode() { return driveMode; }
    public synchronized ControlMode getTalonControlMode() { return talonMode; }
    
    public synchronized void   setMotors(WheelSpeed _wheelSpeed) { wheelSpeed.left = _wheelSpeed.left; wheelSpeed.right = _wheelSpeed.right; setCommandTime(); }
    public synchronized void   setMotors(double _left, double _right) { wheelSpeed.left = _left; wheelSpeed.right = _right; setCommandTime(); }
    public synchronized double getLeftMotor()  { return wheelSpeed.left; }
    public synchronized double getRightMotor() { return wheelSpeed.right; }
    public synchronized double getSpeed() { return (wheelSpeed.left + wheelSpeed.right)/2.0; }

    public synchronized void        setNeutralMode(NeutralMode _neutralMode) { neutralMode = _neutralMode; }
    public synchronized static NeutralMode getNeutralMode()  { return neutralMode; }
    
    public synchronized void    setResetEncoders() { resetEncoders = true; }
    public synchronized boolean getResetEncoders() 
    {	
    	// self-clearing reset on read
    	boolean rv = resetEncoders; 
    	resetEncoders = false; 
    	return rv; 
    }	
    
    public synchronized void   setCommandTime() { commandTime = Timer.getFPGATimestamp(); }
    public synchronized double getCommandTime() { return commandTime; } 
    
    
    // special constant commands
    public static DriveCommand COAST() { return new DriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, NeutralMode.Coast); }
    public static DriveCommand BRAKE() { return new DriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, NeutralMode.Brake); }
    
    
    @Override
    public synchronized String toString() 
    {
    	return String.format("%s, %s, %s, L/R: (%+7.3f, % 7.3f)", driveMode, talonMode, neutralMode, wheelSpeed.left, wheelSpeed.right);
    }
    
    
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        	synchronized (DriveCommand.this)
        	{
	    		put("DriveCommand/driveMode", driveMode.toString() );
	    		put("DriveCommand/talonMode", talonMode.toString() );
	    		put("DriveCommand/left",  wheelSpeed.left );
	       		put("DriveCommand/right", wheelSpeed.right );
	       		put("DriveCommand/brake", (neutralMode == NeutralMode.Coast ? "Coast" : "Brake") );
        	}
        }
    };
    
    public DataLogger getLogger() { return logger; }
    
    
}
