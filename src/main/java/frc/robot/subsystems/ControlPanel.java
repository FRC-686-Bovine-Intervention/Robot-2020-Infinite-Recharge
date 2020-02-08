package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.lib.util.DataLogger;
import frc.robot.loops.Loop;

public class ControlPanel implements Loop {
    // singleton class
    private static ControlPanel instance = null;

    public static ControlPanel getInstance() {
        if (instance == null) {
            instance = new ControlPanel();
        }
        return instance;
    }

    //====================================================
    // Members
    //====================================================
    public TalonSRX panelMaster;
    public ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    public ColorMatch colorMatch = new ColorMatch();

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

    public static double targetRPM = 0;
    public static double kRPMErrorShooting = 360.0, kRPMErrorStopping = 20.0;

    public static Color kBlueTarget =   ColorMatch.makeColor(0.12, 0.40, 0.48);
    public static Color kGreenTarget =  ColorMatch.makeColor(0.16, 0.57, 0.26);
    public static Color kRedTarget =    ColorMatch.makeColor(0.51, 0.34, 0.14);
    public static Color kYellowTarget = ColorMatch.makeColor(0.31, 0.55, 0.12);
    public ColorEnum detectedColorEnum = ColorEnum.UNKNOWN;

    public static enum ColorEnum
    {
        BLUE, GREEN, RED, YELLOW, UNKNOWN;
    }

    public static enum SpinnerStateEnum
    {
        IDLE, ROTATION, SENSOR, WAIT, MANUAL;
    }

    public SpinnerStateEnum SpinnerState = SpinnerStateEnum.IDLE;

    //Ignore this comment

    public ControlPanel() 
    {
        panelMaster = new TalonSRX(Constants.kPanelMasterId);

        //====================================================
        // Configure Deploy Motors
        //====================================================

        // Factory default hardware to prevent unexpected behavior
        panelMaster.configFactoryDefault();

		// configure encoder
		panelMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		panelMaster.setSensorPhase(false); // set so that positive motor input results in positive change in sensor value
        panelMaster.setInverted(true);   // set to have green LEDs when driving forward
        panelMaster.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		
		// set relevant frame periods to be at least as fast as periodic rate
		panelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		panelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		panelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		panelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        panelMaster.selectProfileSlot(kSlotIdx, Constants.kTalonPidIdx); 
        panelMaster.config_kF(kSlotIdx, kKf, Constants.kTalonTimeoutMs); 
        panelMaster.config_kP(kSlotIdx, kKp, Constants.kTalonTimeoutMs); 
        panelMaster.config_kI(kSlotIdx, kKi, Constants.kTalonTimeoutMs); 
        panelMaster.config_kD(kSlotIdx, kKd, Constants.kTalonTimeoutMs);
        panelMaster.configAllowableClosedloopError(kSlotIdx, kAllowableError, Constants.kTalonTimeoutMs);
        
        // current limits
        panelMaster.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        panelMaster.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        panelMaster.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        panelMaster.enableCurrentLimit(true);
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
        panelMaster.set(ControlMode.Velocity, encoderSpeed);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/EncoderSpeed", encoderSpeed);
    }
    

    public double getSpeedError()
    {
        double sensorRPM = encoderUnitsPerFrameToRPM(panelMaster.getSelectedSensorVelocity());
        double errorRPM = sensorRPM - targetRPM;

        SmartDashboard.putNumber("Shooter/SensorRPM", sensorRPM);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/ErrorRPM", errorRPM);

        return Math.abs(errorRPM);
    }

    public void run()
    {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);
        if      (match.color == kBlueTarget)    {detectedColorEnum = ColorEnum.BLUE;}
        else if (match.color == kGreenTarget)   {detectedColorEnum = ColorEnum.GREEN;}
        else if (match.color == kRedTarget)     {detectedColorEnum = ColorEnum.RED;}
        else if (match.color == kYellowTarget)  {detectedColorEnum = ColorEnum.YELLOW;}
        else                                    {detectedColorEnum = ColorEnum.UNKNOWN;}
        SmartDashboard.putNumber("ControlPanel/Red",            detectedColor.red);
        SmartDashboard.putNumber("ControlPanel/Green",          detectedColor.green);
        SmartDashboard.putNumber("ControlPanel/Blue",           detectedColor.blue);
        SmartDashboard.putNumber("ControlPanel/Confidence",     match.confidence*100);
        SmartDashboard.putString("ControlPanel/DetectedColor",  detectedColorEnum.name());
        SmartDashboard.putString("ControlPanel/ConvertedColor", convertColor(detectedColorEnum).name());
        if(!SmartDashboard.getBoolean("ControlPanel/Debug", false)){
            /*
            When sensors activate
            align sensor
            if previous action was nothing then
                rotation control
            else
                position contol
            */
        }
        else
        {

        }
    }

    public void rotationControl()
    {
        /*
        save color
        set motor to 60 rpm * gear ratio
        count number of times saved color goes past
        if number >= 6 then
            stop
        if driver doesn't drive away within 2 seconds then
            do again
        set previous action to rotate
        */
        ColorEnum savedColor = detectedColorEnum;
        panelMaster.set(ControlMode.Velocity, 60);
        int rotation = 0;
        while (rotation >= 6)
        {
            if (detectedColorEnum == savedColor)
            {
                rotation += 1;
            }
        }
        
    }

    public void positionControl()
    {
        /*
        set motor to 60 rpm * gear ratio
        if color == convert FMS color then
            stop
        if driver doesn't drive away within 2 seconds then
            do again
        */
    }

    public ColorEnum convertColor(ColorEnum color)
    {
        switch (color)
        {
            case BLUE:      return ColorEnum.RED;
            case RED:       return ColorEnum.BLUE;
            case YELLOW:    return ColorEnum.GREEN;
            case GREEN:     return ColorEnum.YELLOW;
            default:        return ColorEnum.UNKNOWN;
        }
    }

    public void setupColors()
    {
        colorMatch.addColorMatch(kBlueTarget);
        colorMatch.addColorMatch(kGreenTarget);
        colorMatch.addColorMatch(kRedTarget);
        colorMatch.addColorMatch(kYellowTarget);
    }

    public void stop()
    {
        setSpeed(0);
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