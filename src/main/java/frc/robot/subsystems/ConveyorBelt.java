package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PIDBase.Tolerance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.loops.Loop;
import frc.robot.subsystems.Shooter;
import jdk.dynalink.linker.ConversionComparator;
import frc.robot.Constants;



public class ConveyorBelt implements Loop
{
    public TalonSRX conveyorMaster;
    public VictorSPX conveyorSlave;

	// singleton class
    private static ConveyorBelt instance = null;
    public static ConveyorBelt getInstance() 
	{ 
		if (instance == null) {
            instance = new ConveyorBelt();
		}
		return instance;
    }
	

    public static final int kSlotIdxSpeed = 0;
    public static final int kSlotIdxPos   = 1;

    public static final double kCalMaxEncoderPulsePer100ms = 1950;	// velocity at a max throttle (measured using Phoenix Tuner)
    public static final double kCalMaxPercentOutput 		= 1.0;	// percent output of motor at above throttle (using Phoenix Tuner)

    
	public static final double kKfSpeed = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public static final double kKpSpeed = 5;	   
    public static final double kKiSpeed = 0.0;    
    public static final double kKdSpeed = 10000;	// to resolve any overshoot, start at 10*Kp 
    
    public static final double kKfPos = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
    public static final double kKpPos = 7; // was 7
    public static final double kKiPos = 0;
    public static final double kKdPos = 10000;

    public static final double kKfVel = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
    public static final double kKpVel = 0;
    public static final double kKiVel = 0;
    public static final double kKdVel = 0;
    
    public static final double kQuadEncoderCodesPerRev = 1024;
	public static final double kQuadEncoderUnitsPerRev = 4*kQuadEncoderCodesPerRev;
    public static final double kQuadEncoderStatusFramePeriod = 0.100; // 100 ms
    public static final double kQuadEncoderUnitsPerDeg = kQuadEncoderUnitsPerRev/360;

    public static final double kEncoderUnitsPerInchTower = 2000; //Determined from testing
    
    public static final double kCruiseVelocity = rpmsToEncoderUnitsPerFramePerSec(30.0);		// cruise below top speed; wasa 30
    public static final double timeToCruiseVelocity = 0.1;  // seconds
    public static final double kMaxAcceleration = kCruiseVelocity /timeToCruiseVelocity;
    
    public static final int kAllowableError = (int)rpmToEncoderUnitsPerFrame(15);
    public static final int kAllowableErrorPos = 1;

    public static final int kPeakCurrentLimit = 30;
    public static final int kPeakCurrentDuration = 200;
    public static final int kContinuousCurrentLimit = 20;

    public boolean shooterChecked = false;
    public static final double inchesPerBall = 7;
    public double targetPosInches = 0;
    public static final double posToleranceInches = 1;

    public ConveyorBelt() 
    {
        conveyorMaster = new TalonSRX(Constants.kConveyorbeltMasterID);
        conveyorSlave = new VictorSPX(Constants.kConveyorbeltSlaveID);

        conveyorMaster.configFactoryDefault();
        conveyorSlave.configFactoryDefault();


        conveyorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
        conveyorMaster.setSensorPhase(true);
        conveyorMaster.setInverted(false);

        conveyorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);
        conveyorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);
        conveyorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);
        conveyorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);

        //Config for position PID:
        conveyorMaster.selectProfileSlot(kSlotIdxPos, Constants.kTalonPidIdx);
        conveyorMaster.config_kF(kSlotIdxPos, kKfPos, Constants.kTalonPidIdx);
        conveyorMaster.config_kP(kSlotIdxPos, kKpPos, Constants.kTalonPidIdx);
        conveyorMaster.config_kI(kSlotIdxPos, kKiPos, Constants.kTalonPidIdx);
        conveyorMaster.config_kD(kSlotIdxPos, kKdPos, Constants.kTalonPidIdx);

        //Config for velocity PID:
        conveyorMaster.selectProfileSlot(kSlotIdxSpeed, Constants.kTalonPidIdx);
        conveyorMaster.config_kF(kSlotIdxPos, kKfVel, Constants.kTalonPidIdx);
        conveyorMaster.config_kP(kSlotIdxPos, kKpVel, Constants.kTalonPidIdx);
        conveyorMaster.config_kI(kSlotIdxPos, kKiVel, Constants.kTalonPidIdx);
        conveyorMaster.config_kD(kSlotIdxPos, kKdVel, Constants.kTalonPidIdx);

        //Motion Magic magic
        conveyorMaster.configMotionCruiseVelocity((int)kCruiseVelocity);
        conveyorMaster.configMotionAcceleration((int)kMaxAcceleration);

        //Current limits and stuff
        conveyorMaster.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        conveyorMaster.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        conveyorMaster.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        conveyorMaster.enableCurrentLimit(true);
    

        //Setting up slave
        conveyorSlave.follow(conveyorMaster);
        conveyorSlave.setInverted(false);


        SmartDashboard.putBoolean("Conveyor/Debug", false);
        SmartDashboard.putNumber("Conveyor/Debug/TowerRPM", 0);
        SmartDashboard.putNumber("Conveyor/Debug/TowerPos", 0);
        SmartDashboard.putNumber("Conveyor/Debug/LeftHopperRPM", 0);
        SmartDashboard.putNumber("Conveyor/Debug/RightHopperRPM", 0);
    }

    @Override
    public void onStart(){
        //Do nothing?
    }

    @Override
    public void onLoop(){
        if(!SmartDashboard.getBoolean("Conveyor/Debug", false)){
            SelectedDriverControls driverControls = SelectedDriverControls.getInstance();

            //Starts feeding when shooter has achieved a high enough speed
            if(driverControls.getBoolean(DriverControlsEnum.SHOOT) && shooterChecked){
                setTowerSpeed(60);
            } else {
                //Intentionally ignored while feeding in order to prevent jittering
                shooterChecked = Shooter.getInstance().nearTarget(true);
            }
        } else {

        }
    }

    @Override
    public void onStop() {
        stop();
    }

    
    public void zeroSensors(){
    }

    public void stop()
    {
        setTowerSpeed(0);
    }


    //================================
    //Tower Controls/Functions:
    //================================

    public void setTowerSpeed(double rpm){
        conveyorMaster.set(ControlMode.Velocity, rpmToEncoderUnitsPerFrame(rpm));
    }

    public void setTowerPosition(double inches){
        //Moves the conveyor belt up or down some many inches. Positive moves it up
        conveyorMaster.set(ControlMode.MotionMagic, inchesToEncoderUnitsTower(inches) );
    }

    public void feed(int numberOfBalls){
        targetPosInches = getConveyorPosInches() + (inchesPerBall*(double)numberOfBalls);
        setTowerPosition(targetPosInches);
    }


    public boolean nearTarget(){
        return (Math.abs(targetPosInches-getConveyorPosInches())<=posToleranceInches);
    }

    public double getConveyorPosInches(){
        return encoderUnitsToInchesTower(conveyorMaster.getSelectedSensorPosition());
    }

    private double encoderUnitsToInchesTower(double _units){
        return _units/kEncoderUnitsPerInchTower;
    }

    private int inchesToEncoderUnitsTower(double _inches){
        return (int)(_inches*kEncoderUnitsPerInchTower);
    }

    private int inchesPerSecToEncoderUnitsPerFrame(double ){

    }



    //=============================
    //Hopper Controls/Functions
    //=============================

    private double 







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