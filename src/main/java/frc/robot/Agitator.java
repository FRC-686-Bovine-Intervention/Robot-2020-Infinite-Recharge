package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.subsystems.Shooter;



public class Agitator  
{
    public TalonSRX agitatorMotor;
	// singleton class
	private static Agitator instance = null;
	public static Agitator getInstance() 
	{ 
		if (instance == null) {
			instance = new Agitator();
		}
		return instance;
    }
	
	// public enum AgitatorState { UNINITIALIZED, ARM_BAR_DELAY, ZEROING, RUNNING, ESTOPPED; }
	// private AgitatorState state = AgitatorState.UNINITIALIZED;
	// private AgitatorState nextState = AgitatorState.UNINITIALIZED;
	

    public static final int kSlotIdxSpeed = 0;
    public static final int kSlotIdxPos   = 1;

    public static final double kCalMaxEncoderPulsePer100ms = 1950;	// velocity at a max throttle (measured using Phoenix Tuner)
    public static final double kCalMaxPercentOutput 		= 1.0;	// percent output of motor at above throttle (using Phoenix Tuner)

    
	public static final double kKfSpeed = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public static double kKpSpeed = 5;	   
    public static double kKiSpeed = 0.0;    
    public static double kKdSpeed = 10000;	// to resolve any overshoot, start at 10*Kp 
    
    public static final double kKfPos = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
    public static double kKpPos = 7; // was 7
    public static double kKiPos = 0;
    public static double kKdPos = 10000;
    
    
    public static final double kGearRatio = 21;
    public static final double kQuadEncoderCodesPerRev = 1024;
	public static final double kQuadEncoderUnitsPerRev = 4*kQuadEncoderCodesPerRev;
    public static final double kQuadEncoderStatusFramePeriod = 0.100; // 100 ms
    public static final double kQuadEncoderUnitsPerDeg = kQuadEncoderUnitsPerRev/360;
    
    public static final double kCruiseVelocity = rpmsToEncoderUnitsPerFramePerSec(30.0);		// cruise below top speed; wasa 30
    public static final double timeToCruiseVelocity = 0.1;  // seconds
    public static final double kMaxAcceleration = kCruiseVelocity /timeToCruiseVelocity;
    
    public static final int kAllowableError = (int)rpmToEncoderUnitsPerFrame(15);
    public static final int kAllowableErrorPos = 1;

    public static final int kPeakCurrentLimit = 30;
    public static final int kPeakCurrentDuration = 200;
    public static final int kContinuousCurrentLimit = 20;

    public boolean shooterChecked = false;

    private double targetAngle = 0.0;

    public Agitator() 
    {
        agitatorMotor = new TalonSRX(Constants.kAgitatorTalonId);

        //====================================================
        // Configure Deploy Motors
        //====================================================

        // Factory default hardware to prevent unexpected behavior
        agitatorMotor.configFactoryDefault();

		// configure encoder
		agitatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		agitatorMotor.setSensorPhase(true); // set so that positive motor input results in positive change in sensor value
		agitatorMotor.setInverted(true);   // set to have green LEDs when driving forward
		
		// set relevant frame periods to be at least as fast as perdic rate
		agitatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		agitatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		agitatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		agitatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        agitatorMotor.selectProfileSlot(kSlotIdxSpeed, Constants.kTalonPidIdx); 
        agitatorMotor.config_kF(kSlotIdxSpeed, kKfSpeed, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kP(kSlotIdxSpeed, kKpSpeed, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kI(kSlotIdxSpeed, kKiSpeed, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kD(kSlotIdxSpeed, kKdSpeed, Constants.kTalonTimeoutMs);
        agitatorMotor.configAllowableClosedloopError(kSlotIdxSpeed, kAllowableError, Constants.kTalonTimeoutMs);

        // configure angle loop PID
        agitatorMotor.selectProfileSlot(kSlotIdxPos, Constants.kTalonPidIdx); 
        agitatorMotor.config_kF(kSlotIdxPos, kKfPos, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kP(kSlotIdxPos, kKpPos, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kI(kSlotIdxPos, kKiPos, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kD(kSlotIdxPos, kKdPos, Constants.kTalonTimeoutMs);
        agitatorMotor.configAllowableClosedloopError(kSlotIdxPos, kAllowableErrorPos, Constants.kTalonTimeoutMs);
        
        // Motion Magic
        agitatorMotor.configMotionCruiseVelocity((int)kCruiseVelocity);
        agitatorMotor.configMotionAcceleration((int)kMaxAcceleration);
        
        // current limits
        agitatorMotor.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        agitatorMotor.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        agitatorMotor.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        agitatorMotor.enableCurrentLimit(true);

        zeroSensors();
    }

    public void setSpeed(double rpm)
    {
        agitatorMotor.setNeutralMode(NeutralMode.Coast);
        double encoderSpeed = rpmToEncoderUnitsPerFrame(rpm);
        agitatorMotor.selectProfileSlot(kSlotIdxSpeed, Constants.kTalonPidIdx); 
        agitatorMotor.set(ControlMode.Velocity, encoderSpeed);
        SmartDashboard.putNumber("Agitator/Raw Speed", encoderSpeed);
    }

    public void shootBalls(int balls)
    {
        targetAngle += 60.0 * balls;
        setDegree(targetAngle);
    }

    public void setDegree(double deg)
    {
        agitatorMotor.setNeutralMode(NeutralMode.Brake);
        agitatorMotor.selectProfileSlot(kSlotIdxPos, Constants.kTalonPidIdx); 
        agitatorMotor.set(ControlMode.MotionMagic, degreesToEncoderUnits(deg));
        SmartDashboard.putNumber("Agitator/Raw Angle", degreesToEncoderUnits(deg));
    }

    public double getAngleDegError()
    {
        agitatorMotor.selectProfileSlot(kSlotIdxPos, Constants.kTalonPidIdx); 
        double sensorAngleDeg = encoderUnitsToDegrees(agitatorMotor.getSelectedSensorPosition(Constants.kTalonPidIdx));
        double errorAngleDeg = sensorAngleDeg - targetAngle;

        SmartDashboard.putNumber("Agitator/sensorAngleDeg",sensorAngleDeg);
        SmartDashboard.putNumber("Agitator/targetAngleDeg",targetAngle);
        SmartDashboard.putNumber("Agitator/errorAngleDeg",errorAngleDeg);

        return Math.abs(errorAngleDeg);
    }

    public boolean nearTarget(){
        return getAngleDegError() < kAllowableErrorPos;
    }

    public void setPIDValues(double P, double I, double D)
    {
        kKpSpeed = P;
        kKiSpeed = I;
        kKdSpeed = D;

        // int slot = kSlotIdxSpeed;
        int slot = kSlotIdxPos;
        agitatorMotor.selectProfileSlot(slot, Constants.kTalonPidIdx); 
        agitatorMotor.config_kP(slot, kKpSpeed, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kI(slot, kKiSpeed, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kD(slot, kKdSpeed, Constants.kTalonTimeoutMs);
    }

    public void run()
    {
        SelectedDriverControls driverControls = SelectedDriverControls.getInstance();
        if (driverControls.getBoolean(DriverControlsEnum.SHOOT) && (Shooter.getInstance().nearTarget(true) || shooterChecked))
        {
            setSpeed(60);
            shooterChecked = true;
        }
        else if (driverControls.getBoolean(DriverControlsEnum.UNJAM))
        {
            setSpeed(-30);
            shooterChecked = false;
        } 
        else 
        {
            setSpeed(0);
            shooterChecked = false;
        }
        SmartDashboard.putBoolean("Shooter Checked", shooterChecked);
    }
    
    public void zeroSensors()
    {
        targetAngle = 0.0;
		agitatorMotor.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
    }

    public void stop()
    {
        setSpeed(0);
    }

   	// Talon SRX reports position in rotations while in closed-loop Position mode
    public static double encoderUnitsToRevolutions(int _encoderPosition) {return (double)_encoderPosition / (double)kQuadEncoderUnitsPerRev; }
    public static double encoderUnitsToDegrees(int _encoderPosition) {return encoderUnitsToRevolutions(_encoderPosition) * 360.0; }
    public static int revolutionsToEncoderUnits(double _rev) {return (int)(_rev * kQuadEncoderUnitsPerRev);}
    public static int degreesToEncoderUnits(double _deg) {return (int)(_deg * kQuadEncoderUnitsPerDeg);}

    // Talon SRX reports speed in RPM while in closed-loop Speed mode
    public static double encoderUnitsPerFrameToRPM(int _encoderEdgesPerFrame) { return encoderUnitsToRevolutions(_encoderEdgesPerFrame) * 60.0 / kQuadEncoderStatusFramePeriod; }
    public static int rpmToEncoderUnitsPerFrame(double _rpm) { return (int)(revolutionsToEncoderUnits(_rpm) / 60.0 * kQuadEncoderStatusFramePeriod); }
    public static int rpmsToEncoderUnitsPerFramePerSec(double rpms){return (int)(rpms*(kQuadEncoderUnitsPerRev)*(1.0/60.0)*(1.0/10.0));}
   
}