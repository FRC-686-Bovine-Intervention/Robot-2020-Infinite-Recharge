package frc.robot.subsystems;

import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveCommand.DriveControlMode;
import frc.robot.command_status.DriveState;
import frc.robot.lib.joystick.DriverControlsBase;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Kinematics.WheelSpeed;
import frc.robot.lib.util.PIDController;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.Loop;
import frc.robot.subsystems.Lift.PTOTansmissionState;

/**
 * The robot's drivetrain, which implements the Superstructure abstract class.
 * The drivetrain has several states and builds on the abstract class by
 * offering additional control methods, including control by path and velocity.
 * 
 * @see Subsystem.java
 * 
 * 
 */



public class Drive extends Subsystem
{
	// singleton class
	private static Drive instance = null;
	public static Drive getInstance() 
	{ 
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	// drive commands
	private DriveCommand driveCmd;

	// drive status
	public DriveState driveState;
	
	// velocity heading
	private VelocityHeadingSetpoint velocityHeadingSetpoint = new VelocityHeadingSetpoint();




	// The constructor instantiates all of the drivetrain components when the
	// robot powers up
	private Drive() 
	{
		driveCmd = DriveCommand.COAST();	
		driveState = DriveState.getInstance();
	}

	
	
	/*
	 * Loop to tend to velocity control loops, where Talon SRXs are monitoring the wheel velocities
	 */
	// TODO: move into VelocityHeading class
    private final Loop velocityControlLoop = new Loop() 
    {
        @Override
        public void onStart()
        {
            setOpenLoop(DriveCommand.COAST());
        }

        @Override
        public void onLoop() 
        {
        	switch (driveCmd.getDriveControlMode())
    		{
    			case OPEN_LOOP:
    			case BASE_LOCKED:
    				// states where Talon SRXs are not controlling velocity
    				return;

    			case VELOCITY_SETPOINT:
    				// Nothing to do: Talons SRXs are updating the control loop state
    				return;
    				
    			case VELOCITY_HEADING:
    				// Need to adjust left/right motor velocities to keep desired heading
    				updateVelocityHeading();
    				return;
    				
    			default:
    				System.out.println("Unexpected drive control state: " + driveCmd.getDriveControlMode());
    				break;
    		}
    	}

        @Override
        public void onStop() 
        {
            setOpenLoop(DriveCommand.COAST());
        }
    };

    public Loop getVelocityPIDLoop() { return velocityControlLoop; }
    
    
    /*
     * Main functions to control motors for each DriveControlState
     */
    	
	public void setOpenLoop(DriveCommand cmd) 
	{
		driveCmd.setDriveMode(DriveControlMode.OPEN_LOOP);
		driveCmd.setMotors(cmd.getLeftMotor(), cmd.getRightMotor());
	}

	public void setBaseLockOn() 
	{
		driveCmd.setDriveMode(DriveControlMode.BASE_LOCKED);
	}

	public void setVelocitySetpoint(WheelSpeed _wheelSpeed) 
	{
		driveCmd.setDriveMode(DriveControlMode.VELOCITY_SETPOINT);
		driveCmd.setMotors(_wheelSpeed);
	}

	public void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) 
	{
		driveCmd.setDriveMode(DriveControlMode.VELOCITY_SETPOINT);
		driveCmd.setMotors(left_inches_per_sec, right_inches_per_sec);
	}

	public void setVelocityHeadingSetpoint(double forward_inches_per_sec, double headingSetpointDeg) 
	{
		driveCmd.setDriveMode(DriveControlMode.VELOCITY_HEADING);
		velocityHeadingSetpoint = new VelocityHeadingSetpoint(forward_inches_per_sec, headingSetpointDeg);
		velocityHeadingSetpoint.velocityHeadingPID.reset();
		updateVelocityHeading();
	}
    

	/*
	 * Set/get functions
	 */
    
	public void setCommand(DriveCommand cmd) { driveCmd = cmd; }
	public DriveCommand getCommand() { return driveCmd; }	

    
	/**
	 * VelocityHeadingSetpoints are used to calculate the robot's path given the
	 * speed of the robot in each wheel and the polar coordinates. Especially
	 * useful if the robot is negotiating a turn and to forecast the robot's
	 * location.
	 */
	public static class VelocityHeadingSetpoint 
	{
		private final double speed;
		private final double headingSetpointDeg;

		private PIDController velocityHeadingPID = new PIDController();

		// Constructor for straight line motion
		public VelocityHeadingSetpoint()
		{
			this(0, 0);
		}

		public VelocityHeadingSetpoint(double _speed, double _headingSetpointDeg) 
		{
			speed = _speed;
			headingSetpointDeg = _headingSetpointDeg;
			
			velocityHeadingPID = new PIDController(DriveLoop.kDriveHeadingVelocityKp, DriveLoop.kDriveHeadingVelocityKi, DriveLoop.kDriveHeadingVelocityKd);
			velocityHeadingPID.setOutputRange(-30, 30);
			
			velocityHeadingPID.setSetpoint(headingSetpointDeg);
		}

		public double getSpeed()  { return speed; }
		public double getHeadingSetpointDeg() { return headingSetpointDeg; }
	}
	

	
	/**************************************************************************
	 * VelocityHeading code
	 * (updates VelocitySetpoints in order to follow a heading)
	 *************************************************************************/
    
	private void updateVelocityHeading() 
	{
		// get change in left/right motor speeds based on error in heading
		double diffSpeed = velocityHeadingSetpoint.velocityHeadingPID.calculate( driveState.getHeadingDeg() );
		
		// speed up   left side when robot turns left (actual heading > heading setpoint --> diffSpeed < 0) 
		// slow down right side when robot turns left (actual heading > heading setpoint --> diffSpeed < 0) 
		updateVelocitySetpoint(velocityHeadingSetpoint.getSpeed() - diffSpeed / 2,
				               velocityHeadingSetpoint.getSpeed() + diffSpeed / 2);
	}

	public void resetVelocityHeadingPID()
	{
		// called from DriveLoop, synchronous with changing into VelocityHeading mode
		velocityHeadingSetpoint.velocityHeadingPID.reset();
		velocityHeadingSetpoint.velocityHeadingPID.setSetpoint(velocityHeadingSetpoint.getHeadingSetpointDeg());
	}
	
	
	/**************************************************************************
	 * VelocitySetpoint code
	 * Configures Talon SRXs to desired left/right wheel velocities
	 *************************************************************************/
	
	private void updateVelocitySetpoint(double _left_inches_per_sec, double _right_inches_per_sec) 
	{
		driveCmd.setMotors(_left_inches_per_sec, _right_inches_per_sec);
	}


	// test function -- rotates wheels 1 RPM
	public void testDriveSpeedControl() 
	{
		double  left_inches_per_second = DriveLoop.kDriveWheelCircumInches;
		double right_inches_per_second = DriveLoop.kDriveWheelCircumInches;
		setVelocitySetpoint(left_inches_per_second, right_inches_per_second);
	}


	
	
	
	
	/*
	 * Subsystem overrides(non-Javadoc)
	 * @see frc.robot.subsystems.Subsystem#stop()
	 */
	
	@Override
	public void stop()
	{ 
		setOpenLoop(DriveCommand.COAST()); 
	}

	@Override
	public void zeroSensors() { driveCmd.setResetEncoders(); }


	
	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			try // pathFollowingController doesn't exist until started
			{
				put("Drive/DriveControlModeCmd", driveCmd.getDriveControlMode().toString() );
				put("Drive/TalonControlModeCmd", driveCmd.getTalonControlMode().toString() );
				put("Drive/lMotorCmd", driveCmd.getLeftMotor() );
				put("Drive/rMotorCmd", driveCmd.getRightMotor() );
				put("Drive/BrakeModeCmd", DriveCommand.getNeutralMode().toString() );
				put("VelocityHeading/PIDError",  velocityHeadingSetpoint.velocityHeadingPID.getError() );
				put("VelocityHeading/PIDOutput", velocityHeadingSetpoint.velocityHeadingPID.get() );

//				AdaptivePurePursuitController.getLogger().log();
			} catch (NullPointerException e) {
				// skip logging pathFollowingController when it doesn't exist
			}
        }
    };
    
    public DataLogger getLogger() { return logger; }
    
}