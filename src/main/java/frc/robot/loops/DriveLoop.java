package frc.robot.loops;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.lib.sensors.BNO055;
import frc.robot.lib.sensors.GyroBase;
import frc.robot.lib.sensors.NavX;
import frc.robot.lib.sensors.Pigeon;
import frc.robot.subsystems.Drive;

/*
 * DriveLoop is the interface between Drive.java and the actual hardware.
 * It runs periodically, taking the commands sent by Drive and sending them to the hardware.
 * In this way, Drive.java does not access the hardware directly.  The benefits of this partition are: 
 * 1) Changes to drive hardware only requires changes to DriveLoop, not Drive
 * 2) DriveLoop can be easily replaced for simulation purposes.
 */

public class DriveLoop implements Loop 
{
 	// singleton class
	 private static DriveLoop instance = null;
	 public static DriveLoop getInstance() 
	 { 
		 if (instance == null) {
			 instance = new DriveLoop();
		 }
		 return instance;
	 }
	 
    private static Drive drive;
	private static GyroBase gyro;
    private DriveState driveState;
    
	public final TalonSRX lMotorMaster;
	public final TalonSRX rMotorMaster;
	public final List<BaseMotorController> lMotorSlaves;
	public final List<BaseMotorController> rMotorSlaves;

	private static final int kVelocityControlSlot = 0;
	private static final int kBaseLockControlSlot = 1;

	// Motor Controller Inversions
	
    public static boolean kLeftMotorInverted = true;
    public static boolean kRightMotorInverted = false;
    public static boolean kLeftMotorSensorPhase = true;
    public static boolean kRightMotorSensorPhase = true;

    public static int kDriveTrainCurrentLimit = 25;


	// Wheels
	public static double kDriveWheelCircumInches    = 18.800;
	public static double kTrackLengthInches         = 17.500;
	public static double kTrackWidthInches          = 23.500;
	public static double kTrackEffectiveDiameter    = (kTrackWidthInches * kTrackWidthInches + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;;
	public static double kTrackScrubFactor          = 0.5;

	// Wheel Encoder
	public static double kQuadEncoderGain  = 1.0;	// number of drive shaft rotations per encoder shaft rotation
													// 1.0 if encoder is directly coupled to the drive shaft
	public static int    kQuadEncoderCodesPerRev    = 1024;
	public static int    kQuadEncoderUnitsPerRev    = (int)(4*kQuadEncoderCodesPerRev / kQuadEncoderGain);  ;
	public static double kQuadEncoderStatusFramePeriod = 0.100;	// 100 ms

	// CONTROL LOOP GAINS   
	public static double kDriveSecondsFromNeutralToFull = 0.375;		// decrease acceleration (reduces current, robot tipping)
	public static double kCalEncoderPulsePer100ms = 1400;		// velocity at a nominal throttle (measured using NI web interface)
	public static double kCalPercentOutput 		 = 0.49;	// percent output of motor at above throttle (using NI web interface)
   
   // CONTROL LOOP GAINS
   public static double kFullThrottlePercentOutput = 1.0;	
   public static double kFullThrottleEncoderPulsePer100ms = 2900; 

    // PID gains for drive velocity loop (sent to Talon)
    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 1.0;
    public static double kDriveVelocityKi = 0.00;
    public static double kDriveVelocityKd = 5.0;
    public static double kDriveVelocityKf = kCalPercentOutput * 1023.0 / kCalEncoderPulsePer100ms;
    public static int    kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.25; //0.375 - time to get to full speed
    public static int    kDriveVelocityAllowableError = 0;

    // PID gains for drive base lock loop
    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 0.5;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int    kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int    kDriveBaseLockAllowableError = 10;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVelocityKp = 4.0;
    public static double kDriveHeadingVelocityKi = 0.0;
    public static double kDriveHeadingVelocityKd = 50.0;
    
    // Point Turn constants
    public static double kPointTurnKp = 0.05;
    public static double kPointTurnKd = 0.50;
    public static double kPointTurnKi = 0.00;
    public static double kPointTurnKf = 0.00;
    public static double kPointTurnCompletionToleranceDeg = 3.0;
    public static double kPointTurnMaxOutput = 0.7; 
    
    // Path following constants
    public static double kPathFollowingMaxVel    = 72.0; // inches/sec  		
    public static double kPathFollowingAccelTime = 0.5;		// sec to reach max velocity
    public static double kPathFollowingMaxAccel  = kPathFollowingMaxVel / kPathFollowingAccelTime; // inches/sec^2
    public static double kPathFollowingLookahead = 24.0; // inches
    public static double kPathFollowingCompletionTolerance = 4.0; 



	private DriveLoop() 
	{
		drive = Drive.getInstance();
		driveState = DriveState.getInstance();
		
		/*****************************************************************
		 * Configure Master Motor Controllers
		 *****************************************************************/
		lMotorMaster = new TalonSRX(Constants.kLeftMotorMasterTalonId);
		rMotorMaster = new TalonSRX(Constants.kRightMotorMasterTalonId);
        
		// Get status at 100Hz (faster than default 50 Hz)
		lMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kTalonTimeoutMs);
		rMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kTalonTimeoutMs);

		lMotorMaster.set(ControlMode.PercentOutput, 0.0);
		rMotorMaster.set(ControlMode.PercentOutput, 0.0);
		lMotorMaster.setNeutralMode(NeutralMode.Coast);
		rMotorMaster.setNeutralMode(NeutralMode.Coast);

		lMotorMaster.configOpenloopRamp(kDriveSecondsFromNeutralToFull, Constants.kTalonTimeoutMs);		
		rMotorMaster.configOpenloopRamp(kDriveSecondsFromNeutralToFull, Constants.kTalonTimeoutMs);		
		
		// Set up the encoders
		lMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);	// configure for closed-loop PID
		rMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		lMotorMaster.setSensorPhase(kLeftMotorSensorPhase);
		rMotorMaster.setSensorPhase(kRightMotorSensorPhase);
		lMotorMaster.setInverted(kLeftMotorInverted);
		rMotorMaster.setInverted(kRightMotorInverted);
		
		// Load velocity control gains
		lMotorMaster.config_kF(kVelocityControlSlot, kDriveVelocityKf, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kP(kVelocityControlSlot, kDriveVelocityKp, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kI(kVelocityControlSlot, kDriveVelocityKi, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kD(kVelocityControlSlot, kDriveVelocityKd, Constants.kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kVelocityControlSlot, kDriveVelocityIZone, Constants.kTalonTimeoutMs);

		rMotorMaster.config_kF(kVelocityControlSlot, kDriveVelocityKf, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kP(kVelocityControlSlot, kDriveVelocityKp, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kI(kVelocityControlSlot, kDriveVelocityKi, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kD(kVelocityControlSlot, kDriveVelocityKd, Constants.kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kVelocityControlSlot, kDriveVelocityIZone, Constants.kTalonTimeoutMs);
		
		lMotorMaster.configAllowableClosedloopError(kVelocityControlSlot, kDriveVelocityAllowableError, Constants.kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kVelocityControlSlot, kDriveVelocityAllowableError, Constants.kTalonTimeoutMs);

		
		// Load base lock control gains
		lMotorMaster.config_kF(kBaseLockControlSlot, kDriveBaseLockKf, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kP(kBaseLockControlSlot, kDriveBaseLockKp, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kI(kBaseLockControlSlot, kDriveBaseLockKi, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kD(kBaseLockControlSlot, kDriveBaseLockKd, Constants.kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kBaseLockControlSlot, kDriveBaseLockIZone, Constants.kTalonTimeoutMs);

		rMotorMaster.config_kF(kBaseLockControlSlot, kDriveBaseLockKf, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kP(kBaseLockControlSlot, kDriveBaseLockKp, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kI(kBaseLockControlSlot, kDriveBaseLockKi, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kD(kBaseLockControlSlot, kDriveBaseLockKd, Constants.kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kBaseLockControlSlot, kDriveBaseLockIZone, Constants.kTalonTimeoutMs);

		lMotorMaster.configAllowableClosedloopError(kBaseLockControlSlot, kDriveBaseLockAllowableError, Constants.kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kBaseLockControlSlot, kDriveBaseLockAllowableError, Constants.kTalonTimeoutMs);
		
		lMotorMaster.configOpenloopRamp(kDriveVelocityRampRate, 0);
		rMotorMaster.configOpenloopRamp(kDriveVelocityRampRate, 0);

		/*****************************************************************
		 * Configure Slave Motor Controllers
		 *****************************************************************/
		lMotorSlaves = new ArrayList<BaseMotorController>();	
		rMotorSlaves = new ArrayList<BaseMotorController>();	
		lMotorSlaves.add(new VictorSPX(Constants.kLeftMotorSlave1TalonId));
		rMotorSlaves.add(new VictorSPX(Constants.kRightMotorSlave1TalonId));			
		
        for (BaseMotorController lMotorSlave : lMotorSlaves) 
        {
    		lMotorSlave.follow(lMotorMaster);	// give slave the TalonID of it's master
    		lMotorSlave.setNeutralMode(NeutralMode.Coast);
    		lMotorSlave.setInverted(kLeftMotorInverted);
        }
        for (BaseMotorController rMotorSlave : rMotorSlaves) 
        {
    		rMotorSlave.follow(rMotorMaster);	// give slave the TalonID of it's master
    		rMotorSlave.setNeutralMode(NeutralMode.Coast);
    		rMotorSlave.setInverted(kRightMotorInverted);
        }
        
		/*****************************************************************
		 * Set Initial Motor Settings
		 *****************************************************************/
		DriveCommand neutralCmd = DriveCommand.COAST();
		setControlMode(neutralCmd);
		setMotors(neutralCmd);
		setNeutralMode(neutralCmd);
		resetEncoders(neutralCmd);        

	
		/*****************************************************************
		 * Select which Gyro is installed
		 *****************************************************************/
		// select which gyro is installed
		switch (Constants.GyroSelection)
		{
		case BNO055:
			System.out.println("Selected gyro = BNO055");
			gyro = BNO055.getInstance();
			break;
		case NAVX:
			System.out.println("Selected gyro = NavX");
			gyro = NavX.getInstance();
			break;
		case PIGEON:
		default:
			System.out.println("Selected gyro = Pigeon");
			gyro = Pigeon.getInstance();
			break;
		}

	}
	
	
	@Override public void onStart()
	{
		// nothing
	}

	@Override public void onLoop()
	{
		// get status from hardware
		getStatus();
		
		// send new commands to hardware
		sendCommands();
	}

	@Override public void onStop()
	{
		stopMotors();
	}

	private void stopMotors()
	{
		drive.setCommand(DriveCommand.COAST());		// override any incoming commands 
		sendCommands();
	}

	private void getStatus()
	{
		synchronized (driveState)	// lock DriveState until we update it, so that objects reading DriveState don't get partial updates	
		{
			// get Talon control & brake modes (assume right motor is configured identically)
			driveState.setTalonControlMode( lMotorMaster.getControlMode() );
			driveState.setNeutralMode( DriveCommand.getNeutralMode() );
			
			// get encoder values from hardware, set in Drive
			driveState.setLeftDistanceInches(  encoderUnitsToInches( lMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ) ));
			driveState.setRightDistanceInches( encoderUnitsToInches( rMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ) ));
	
			driveState.setLeftSpeedInchesPerSec(  encoderUnitsPerFrameToInchesPerSecond( lMotorMaster.getSelectedSensorVelocity(  Constants.kTalonPidIdx  ) ));
			driveState.setRightSpeedInchesPerSec( encoderUnitsPerFrameToInchesPerSecond( rMotorMaster.getSelectedSensorVelocity(  Constants.kTalonPidIdx  ) ));
				
			/*
			 * measured angle decreases with clockwise rotation
			 * it should increase with clockwise rotation (according to
			 * documentation, and standard right hand rule convention
			 * negate it here to correct
			 */
			driveState.setHeadingDeg( gyro.getHeadingDeg() );
	
			driveState.setMotorCurrent(lMotorMaster.getOutputCurrent(), rMotorMaster.getOutputCurrent() );
			driveState.setMotorPIDError(lMotorMaster.getClosedLoopError( Constants.kTalonPidIdx ), rMotorMaster.getClosedLoopError( Constants.kTalonPidIdx ) );
	
	        switch (driveState.getTalonControlMode())
	        {
	        	case PercentOutput: 
	        		driveState.setMotorStatus(lMotorMaster.getMotorOutputPercent(), rMotorMaster.getMotorOutputPercent() );
	                break;
	
	        	case Position:
	        		driveState.setMotorStatus(lMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ), rMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ) );
	        		break;
	        		
	        	case Velocity:
	        		driveState.setMotorStatus(lMotorMaster.getSelectedSensorVelocity( Constants.kTalonPidIdx ), rMotorMaster.getSelectedSensorVelocity( Constants.kTalonPidIdx ) );
	        		break;
	        		
	        	case Disabled:
	        	default:
	        		driveState.setMotorStatus(lMotorMaster.getMotorOutputPercent(), rMotorMaster.getMotorOutputPercent() );
	        		break;
			}
		}
	}
		
	private void sendCommands()
	{
		DriveCommand newCmd = drive.getCommand();
		
		// Watchdog timer  
		double currentTime = Timer.getFPGATimestamp();
		if (currentTime - newCmd.getCommandTime() > Constants.kDriveWatchdogTimerThreshold)
		{
			// Halt robot if new command hasn't been sent in a while
			stopMotors();
			return;
		}
				
		synchronized(newCmd)	// lock DriveCommand so no one changes it under us while we are sending the commands
		{
			setControlMode(newCmd);
			setMotors(newCmd);
			setNeutralMode(newCmd);
			resetEncoders(newCmd);
		}
	}
	
	
	private void setControlMode(DriveCommand newCmd)
    {
		ControlMode newMode = newCmd.getTalonControlMode();
		
		if (newMode != driveState.getTalonControlMode())
		{
	        switch (newMode)
	        {
	        	case PercentOutput: 
	                break;
	
	        	case Position:
	    			lMotorMaster.selectProfileSlot(kBaseLockControlSlot, Constants.kTalonPidIdx);
	    			rMotorMaster.selectProfileSlot(kBaseLockControlSlot, Constants.kTalonPidIdx);

	        		lMotorMaster.set(ControlMode.Position, (double)(lMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx )) );
	        		rMotorMaster.set(ControlMode.Position, (double)(rMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx )) );
	        		break;
	        		
	        	case Velocity:
	        		lMotorMaster.selectProfileSlot(kVelocityControlSlot, Constants.kTalonPidIdx);
	        		rMotorMaster.selectProfileSlot(kVelocityControlSlot, Constants.kTalonPidIdx);
	        		break;
	        		
	        	case Disabled:
	        	default:
	        		break;
	        }
		}
	}
	
	
	
	private void setNeutralMode(DriveCommand newCmd)
	{
		NeutralMode newNeutral = DriveCommand.getNeutralMode();
		setNeutralMode(newNeutral);
	}
	
	
	private void setNeutralMode(NeutralMode newNeutral) 
	{
		if (newNeutral != driveState.getNeutralMode()) 
		{
			lMotorMaster.setNeutralMode(newNeutral);
			rMotorMaster.setNeutralMode(newNeutral);
			for (BaseMotorController lMotorSlave : lMotorSlaves)
				lMotorSlave.setNeutralMode(newNeutral);
			for (BaseMotorController rMotorSlave : rMotorSlaves)
				rMotorSlave.setNeutralMode(newNeutral);
		}
	}
	
	
		
	private void setMotors(DriveCommand newCmd)
    {
		double lMotorCtrl = newCmd.getLeftMotor();
		double rMotorCtrl = newCmd.getRightMotor();
		
        switch (newCmd.getTalonControlMode())	// assuming new mode is already configured
        {
        	case PercentOutput:
        		// DriveCommand given in range +/-1, with 1 representing full throttle
        		lMotorMaster.set(ControlMode.PercentOutput, lMotorCtrl);
        		rMotorMaster.set(ControlMode.PercentOutput, rMotorCtrl);
        		break;

        	case Position:
        		// no changes to position set in setControlMode()
        		break;
        		
        	case Velocity:
        		// DriveCommand given in inches/sec
        		// Talon SRX needs RPM in closed-loop mode.
        		// convert inches/sec to encoder edges per 100ms
           		lMotorMaster.set(ControlMode.Velocity, inchesPerSecondToEncoderUnitsPerFrame(lMotorCtrl)); 
        		rMotorMaster.set(ControlMode.Velocity, inchesPerSecondToEncoderUnitsPerFrame(rMotorCtrl));
        		break;
        		
        	case Disabled:
        	default:
        		lMotorMaster.set(ControlMode.Disabled, 0);
        		rMotorMaster.set(ControlMode.Disabled, 0);
        		break;
        }
	}

	// Talon SRX reports position in rotations while in closed-loop Position mode
	public static double encoderUnitsToInches(int _encoderPosition) {	return (double)_encoderPosition / (double)kQuadEncoderUnitsPerRev  * kDriveWheelCircumInches; }
	public static int inchesToEncoderUnits(double _inches) { return (int)(_inches / kDriveWheelCircumInches * kQuadEncoderUnitsPerRev); }

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	public static double encoderUnitsPerFrameToInchesPerSecond(int _encoderEdgesPerFrame) { return encoderUnitsToInches(_encoderEdgesPerFrame) / kQuadEncoderStatusFramePeriod; }
	public static int inchesPerSecondToEncoderUnitsPerFrame(double _inchesPerSecond) { return (int)(inchesToEncoderUnits(_inchesPerSecond) * kQuadEncoderStatusFramePeriod); }

	
	

	private void resetEncoders(DriveCommand newCmd)
	{
		if (newCmd.getResetEncoders())
		{
			SensorCollection collection = lMotorMaster.getSensorCollection();
			collection.setQuadraturePosition(0, Constants.kTalonTimeoutMs);
			
			collection = rMotorMaster.getSensorCollection();
			collection.setQuadraturePosition(0, Constants.kTalonTimeoutMs);
			
			gyro.zeroSensor();  
			// calibration to desired initial pose is done in RobotState.reset() called from Robot.autonomousInit()  
		}
	}	


};
