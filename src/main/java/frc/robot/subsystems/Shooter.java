package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.joystick.DriverAxisEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Vector2d;
import frc.robot.loops.Loop;

public class Shooter implements Loop {
    // singleton class
    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    //====================================================
    // Members
    //====================================================
    public TalonSRX shooterMotor;
    public VictorSPX shooterSlave;
    public Limelight camera = Limelight.getInstance();
    public double speed;

    //====================================================
    // Constants
    //====================================================
    
    public final int kSlotIdx = 0;

    public final double kCalMaxEncoderPulsePer100ms = 33300;	// velocity at a max throttle (measured using Phoenix Tuner)
    public final double kCalMaxPercentOutput 		= 1.0;	// percent output of motor at above throttle (using Phoenix Tuner)

	public final double kKf = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public final double kKp = 0.03;	   
	public final double kKd = 2.0625;	// to resolve any overshoot, start at 10*Kp 
	public final double kKi = 0.0;    

	public static double kQuadEncoderCodesPerRev = 1024;
	public static double kQuadEncoderUnitsPerRev = 4*kQuadEncoderCodesPerRev;
	public static double kQuadEncoderStatusFramePeriod = 0.100; // 100 ms

    public final int kAllowableError = (int)rpmToEncoderUnitsPerFrame(10);

    public final int kPeakCurrentLimit = 50;
    public final int kPeakCurrentDuration = 200;
    public final int kContinuousCurrentLimit = 30;
    public final int kSliderMax = 200;

    public double kHighGoalHeight = 76.75;
    public double kLowGoalHeight = 41.5;
    public double kCameraHeight = 23;
    public double kCameraAngle = 37;
    public double kFrontToCameraDist = 27.5;

    public static double targetRPM = 0;
    public static double kRPMErrorShooting = 360.0, kRPMErrorStopping = 20.0;

    public static enum GoalEnum
    {
        LOW_GOAL, HIGH_GOAL, BUNNY_HIGH_GOAL;
    }
 
    // Distance vs. RPM Tables
    public double bunnyHighGoalTable[][] = {
        {14,    3400},
        {24,    3050},
        {34,    3000}   };

    public double highGoalTable[][] = {
        {14,    3400},
        {24,    2950},
        {34,    2850},
        {42,    2925},
        {50,    3000},
        {60,    3125},
        {67,    3250}   };
    
    public double lowGoalTable[][] = {
        {8,    1950},
        {15,   2075},
        {26,   2350}};


    public Shooter() 
    {
        shooterMotor = new TalonSRX(Constants.kShooterTalonId);
        shooterSlave = new VictorSPX(Constants.kShooterSlaveId);

        //====================================================
        // Configure Deploy Motors
        //====================================================

        // Factory default hardware to prevent unexpected behavior
        shooterMotor.configFactoryDefault();

		// configure encoder
		shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		shooterMotor.setSensorPhase(false); // set so that positive motor input results in positive change in sensor value
        shooterMotor.setInverted(true);   // set to have green LEDs when driving forward
        shooterMotor.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		
		// set relevant frame periods to be at least as fast as periodic rate
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        shooterMotor.selectProfileSlot(kSlotIdx, Constants.kTalonPidIdx); 
        shooterMotor.config_kF(kSlotIdx, kKf, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kP(kSlotIdx, kKp, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kI(kSlotIdx, kKi, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kD(kSlotIdx, kKd, Constants.kTalonTimeoutMs);
        shooterMotor.configAllowableClosedloopError(kSlotIdx, kAllowableError, Constants.kTalonTimeoutMs);
        
        // current limits
        shooterMotor.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        shooterMotor.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        shooterMotor.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        shooterMotor.enableCurrentLimit(true);

        // slave stuff
        shooterSlave.follow(shooterMotor);
        shooterSlave.setInverted(false);

        
    }


    //Loop Functions
    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {

    }

    @Override
    public void onStop() {
    }


    // Talon SRX reports position in rotations while in closed-loop Position mode
	public static double encoderUnitsToRevolutions(int _encoderPosition) {	return (double)_encoderPosition / (double)kQuadEncoderUnitsPerRev; }
	public static int revolutionsToEncoderUnits(double _rev) { return (int)(_rev * kQuadEncoderUnitsPerRev); }

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	public static double encoderUnitsPerFrameToRPM(int _encoderEdgesPerFrame) { return encoderUnitsToRevolutions(_encoderEdgesPerFrame) * 60.0 / kQuadEncoderStatusFramePeriod; }
	public static int rpmToEncoderUnitsPerFrame(double _rpm) { return (int)(revolutionsToEncoderUnits(_rpm) / 60.0 * kQuadEncoderStatusFramePeriod); }



    public void setSpeed(double rpm)
    {
        targetRPM = rpm;
        double encoderSpeed = rpmToEncoderUnitsPerFrame(targetRPM);
        shooterMotor.set(ControlMode.Velocity, encoderSpeed);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/EncoderSpeed", encoderSpeed);
    }
    

    public double getSpeedError()
    {
        double sensorRPM = encoderUnitsPerFrameToRPM(shooterMotor.getSelectedSensorVelocity());
        double errorRPM = sensorRPM - targetRPM;

        SmartDashboard.putNumber("Shooter/SensorRPM", sensorRPM);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/ErrorRPM", errorRPM);

        return Math.abs(errorRPM);
    }

    public boolean nearTarget(boolean shooting){
        if (shooting){
            return getSpeedError() < kRPMErrorShooting;
        } else {
            return getSpeedError() <kRPMErrorStopping;
        }
    }

    public void run()
    {
        // SelectedDriverControls driverControls = SelectedDriverControls.getInstance();
 
        // GoalEnum goal = GoalEnum.HIGH_GOAL;
        // if (driverControls.getBoolean(DriverControlsEnum.TARGET_LOW))
        // {
        //     goal = GoalEnum.LOW_GOAL;
        //     Limelight.getInstance().setPipeline(1);
        // } else {
        //     Limelight.getInstance().setPipeline(0);
        // }



        if(SmartDashboard.getBoolean("Shooter/Debug", false)){
            setSpeed(SmartDashboard.getNumber("Shooter/RPM", 0));
            SmartDashboard.putNumber("Shooter/SensedRPM", encoderUnitsPerFrameToRPM(shooterMotor.getSelectedSensorVelocity()));
        }

        // if (!SmartDashboard.getBoolean("Shooter/Debug", false))
        // {
        //     if (driverControls.getBoolean(DriverControlsEnum.SHOOT))
        //     {
        //         setTarget(goal);
        //     }
        //     else
        //     {
        //         stop();
        //     }
        // }
        // else
        // {
        //     setSpeed(SmartDashboard.getNumber("Shooter/RPM", 0));
        //     double distance = handleDistance(camera.getTargetVerticalAngleRad(), goal)-kFrontToCameraDist;
        //     if (camera.getIsTargetFound())
        //     {  
        //         SmartDashboard.putNumber("Shooter/Distance", distance);
        //     }
        // }
        // SmartDashboard.putBoolean("Shooter/Found Target", camera.getIsTargetFound());
    }

    public void setTarget(GoalEnum goal)
    {
        SelectedDriverControls driverControls = SelectedDriverControls.getInstance();

        double goalTable[][] = handleGoalTable(goal);
        double distance = handleDistance(camera.getTargetVerticalAngleRad(), goal)-kFrontToCameraDist;
        int keyL = getLinear(distance, goalTable);
        double shooterCorrection = -driverControls.getAxis(DriverAxisEnum.SHOOTER_SPEED_CORRECTION)*kSliderMax;
        double nominalSpeed = handleLinear(distance, goalTable[keyL][0], goalTable[keyL+1][0], goalTable[keyL][1], goalTable[keyL+1][1]);
        speed = nominalSpeed + shooterCorrection;
        setSpeed(speed);
        SmartDashboard.putString("Shooter/Goal", goal.name());
        if (camera.getIsTargetFound())
        {       
            SmartDashboard.putNumber("Shooter/Distance", distance);
        }        
        SmartDashboard.putNumber("Shooter/CorrectionSpeed", shooterCorrection);
        SmartDashboard.putNumber("Shooter/NominalSpeed", nominalSpeed);
        SmartDashboard.putNumber("Shooter/Target Angle Deg", camera.getTargetVerticalAngleRad() * Vector2d.radiansToDegrees);
        SmartDashboard.putBoolean("Shooter/Found Target", camera.getIsTargetFound());
    }

    public void stop()
    {
        setSpeed(0);
    }

    public double handleDistance (double angleRad, GoalEnum goal)
    {
        if (goal == GoalEnum.LOW_GOAL)
        {
            return (kLowGoalHeight-kCameraHeight)/(Math.tan(angleRad+(kCameraAngle * Vector2d.degreesToRadians)));
        }
        else
        {
            return (kHighGoalHeight-kCameraHeight)/(Math.tan(angleRad+(kCameraAngle * Vector2d.degreesToRadians)));
        }
    }

    public double[][] handleGoalTable (GoalEnum goal)
    {
        switch (goal)
        {
            case LOW_GOAL:          return lowGoalTable;
            case BUNNY_HIGH_GOAL:   return bunnyHighGoalTable;
            case HIGH_GOAL:         return highGoalTable;
            default:                return highGoalTable;
        }
    }

    public int getLinear (double d, double table[][])
    {
        double distance = Math.max(Math.min(d, table[table.length-1][0]), table[0][0]);
        int k;
        for (k=0;k<table.length;k++)
        {
            if (distance <= table[k][0])
            {
                break;
            }
        }
        return Math.max(k-1, 0);
    }

    public double handleLinear (double d, double dL, double dH, double sL, double sH)
    {
        return (sH-sL)*Math.min((d-dL)/(dH-dL),1)+sL;
    }
    



    
	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
            // put("Shooter/Deploy/targetPosition", targetPosition.toString());

		}
	}; 
    
	public DataLogger getLogger()
	{
		return logger;
    }
}