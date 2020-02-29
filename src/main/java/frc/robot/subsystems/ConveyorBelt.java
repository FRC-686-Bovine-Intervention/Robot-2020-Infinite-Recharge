package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.loops.Loop;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;



public class ConveyorBelt implements Loop
{

	// singleton class
    private static ConveyorBelt instance = null;
    public static ConveyorBelt getInstance() 
	{ 
		if (instance == null) {
            instance = new ConveyorBelt();
		}
		return instance;
    }

    //public TalonSRX conveyorMaster, kickerMotor;
    public VictorSPX conveyorSlave, rightHopperMotor, conveyorMaster, kickerMotorMaster, kickerMotorSlave;
    public DigitalInput entranceProximitySensor, exitProximitySensor;
    public TalonSRX leftHopperMotor;

	
    public static final int kSlotIdxSpeed = 0;
    public static final int kSlotIdxPos   = 1;

    public static final double kCalMaxEncoderPulsePer100ms = 1950;	// velocity at a max throttle (measured using Phoenix Tuner)
    public static final double kCalMaxPercentOutput 		= 1.0;	// percent output of motor at above throttle (using Phoenix Tuner)

    
	public static final double kKfSpeedTower = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public static final double kKpSpeedTower = 5;	   
    public static final double kKiSpeedTower = 0.0;    
    public static final double kKdSpeedTower = 10000;	// to resolve any overshoot, start at 10*Kp 
    
    public static final double kKfPosTower = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
    public static final double kKpPosTower = 7; // was 7
    public static final double kKiPosTower = 0;
    public static final double kKdPosTower = 10000;

    public static final double kKfSpeedKicker = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
    public static final double kKpSpeedKicker = 0;
    public static final double kKiSpeedKicker = 0;
    public static final double kKdSpeedKicker = 0;
    
    public static final double kQuadEncoderCodesPerRev = 1024;
	public static final double kQuadEncoderUnitsPerRev = 4*kQuadEncoderCodesPerRev;
    public static final double kQuadEncoderStatusFramePeriod = 0.100; // 100 ms
    public static final double kQuadEncoderUnitsPerDeg = kQuadEncoderUnitsPerRev/360;

    public static final double kEncoderUnitsPerInchTower = 2000; //Determined from testing
    public static final double kEncoderUnitsPerRevKicker = 3000;
    
    public static final double kCruiseVelocity = ipsToEncoderUnitsPerFrameTower(7.0);		// cruise below top speed; wasa 30
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


    //Temp Code stuff
    public int storageCount = 0;
    private RisingEdgeDetector entranceEdge = new RisingEdgeDetector();
    private double backupStartTime = 0;
    private static final double backupSeconds = 0.5;




    public RisingEdgeDetector shootDetector = new RisingEdgeDetector();

    public ConveyorBelt() 
    {
        entranceProximitySensor = new DigitalInput(Constants.kEntranceProximityID);
        exitProximitySensor = new DigitalInput(Constants.kExitProximityID);

        // conveyorMaster = new TalonSRX(Constants.kConveyorbeltMasterID);
        // kickerMotor = new TalonSRX(Constants.kConveyorKickerID);
        kickerMotorMaster = new VictorSPX(Constants.kConveyorKickerMasterID);
        kickerMotorSlave = new VictorSPX(Constants.kConveyorKickerSlaveID);
        conveyorMaster = new VictorSPX(Constants.kConveyorbeltMasterID);
        conveyorSlave = new VictorSPX(Constants.kConveyorbeltSlaveID);
        leftHopperMotor = new TalonSRX(Constants.kConveyorHopperLeftId);
        rightHopperMotor = new VictorSPX(Constants.kConveyorHopperRightID);


        conveyorMaster.configFactoryDefault();
        conveyorSlave.configFactoryDefault();
        leftHopperMotor.configFactoryDefault();
        rightHopperMotor.configFactoryDefault();
        kickerMotorMaster.configFactoryDefault();
        kickerMotorSlave.configFactoryDefault();

        conveyorMaster.setInverted(true);
        kickerMotorMaster.setInverted(true);
        rightHopperMotor.setInverted(true);
        leftHopperMotor.setInverted(true);


        // conveyorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
        // conveyorMaster.setSensorPhase(true);
        // conveyorMaster.setInverted(false);

        // conveyorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);
        // conveyorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);
        // conveyorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);
        // conveyorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);

        // //Config for position PID:
        // conveyorMaster.selectProfileSlot(kSlotIdxPos, Constants.kTalonPidIdx);
        // conveyorMaster.config_kF(kSlotIdxPos, kKfPosTower, Constants.kTalonPidIdx);
        // conveyorMaster.config_kP(kSlotIdxPos, kKpPosTower, Constants.kTalonPidIdx);
        // conveyorMaster.config_kI(kSlotIdxPos, kKiPosTower, Constants.kTalonPidIdx);
        // conveyorMaster.config_kD(kSlotIdxPos, kKdPosTower, Constants.kTalonPidIdx);

        // //Config for velocity PID:
        // conveyorMaster.selectProfileSlot(kSlotIdxSpeed, Constants.kTalonPidIdx);
        // conveyorMaster.config_kF(kSlotIdxSpeed, kKfSpeedTower, Constants.kTalonPidIdx);
        // conveyorMaster.config_kP(kSlotIdxSpeed, kKpSpeedTower, Constants.kTalonPidIdx);
        // conveyorMaster.config_kI(kSlotIdxSpeed, kKiSpeedTower, Constants.kTalonPidIdx);
        // conveyorMaster.config_kD(kSlotIdxSpeed, kKdSpeedTower, Constants.kTalonPidIdx);

        // //Motion Magic magic
        // conveyorMaster.configMotionCruiseVelocity((int)kCruiseVelocity);
        // conveyorMaster.configMotionAcceleration((int)kMaxAcceleration);

        // //Current limits and stuff
        // conveyorMaster.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        // conveyorMaster.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        // conveyorMaster.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        // conveyorMaster.enableCurrentLimit(true);
    

        //Setting up slaves
        conveyorSlave.follow(conveyorMaster);
        conveyorSlave.setInverted(false);

        kickerMotorSlave.follow(kickerMotorMaster);
        kickerMotorSlave.setInverted(false);


        // //===============
        // //Kicker Config:
        // //===============
        // kickerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
        // kickerMotor.setSensorPhase(true);
        // kickerMotor.setInverted(false);

        // kickerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);
        // kickerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);
        // kickerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);
        // kickerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, (int)(1000*Constants.kLoopDt), Constants.kTalonTimeoutMs);

        // //Config for velocity PID:
        // kickerMotor.selectProfileSlot(kSlotIdxSpeed, Constants.kTalonPidIdx);
        // kickerMotor.config_kF(kSlotIdxSpeed, kKfSpeedKicker, Constants.kTalonPidIdx);
        // kickerMotor.config_kP(kSlotIdxSpeed, kKpSpeedKicker, Constants.kTalonPidIdx);
        // kickerMotor.config_kI(kSlotIdxSpeed, kKiSpeedKicker, Constants.kTalonPidIdx);
        // kickerMotor.config_kD(kSlotIdxSpeed, kKdSpeedKicker, Constants.kTalonPidIdx);

        // //Current limits and stuff
        // kickerMotor.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        // kickerMotor.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        // kickerMotor.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        // kickerMotor.enableCurrentLimit(true);



        SmartDashboard.putBoolean("Conveyor/Debug", false);
        // SmartDashboard.putNumber("Conveyor/Debug/SetTowerIPS", 0);
        // SmartDashboard.putNumber("Conveyor/Debug/SetTowerPosInches", 0);

        // SmartDashboard.putNumber("Conveyor/Debug/SensedTowerIPS", 0);
        // SmartDashboard.putNumber("Conveyor/Debug/SensedTowerPosInches", 0);

        SmartDashboard.putNumber("Conveyor/Debug/SetLeftHopperPercent", 0);
        SmartDashboard.putNumber("Conveyor/Debug/SetRightHopperPercent", 0);

        // SmartDashboard.putNumber("Conveyor/Debug/SetKickerRPM", 0);
        // SmartDashboard.putNumber("Conveyor/Debug/KickerSensedRPM", 0);


        SmartDashboard.putNumber("Conveyor/Debug/SetTowerPercent", 0);
        SmartDashboard.putNumber("Conveyor/Debug/SetKickerPercent", 0);
    }

    @Override
    public void onStart(){
        //Do nothing?
        storageCount=0;
    }

    @Override
    public void onLoop(){
        if(!SmartDashboard.getBoolean("Conveyor/Debug", false)){
            SelectedDriverControls driverControls = SelectedDriverControls.getInstance();
            if(driverControls.getBoolean(DriverControlsEnum.RESET)){
                stopHopper();
                kickerMotorMaster.set(ControlMode.PercentOutput, 0.0);
                conveyorMaster.set(ControlMode.PercentOutput, 0.0);
            } else if(driverControls.getBoolean(DriverControlsEnum.REVERSE_BELTS)){
                kickerMotorMaster.set(ControlMode.PercentOutput, 0.0);
                conveyorMaster.set(ControlMode.PercentOutput, -Constants.kConveyorBackUpPercent);
                reverseHopper();   
                storageCount = 0;             
            } else {
                if(shootDetector.update(driverControls.getBoolean(DriverControlsEnum.SHOOT))){
                    kickerMotorMaster.set(ControlMode.PercentOutput, Constants.kKickerShootPercent);
                    conveyorMaster.set(ControlMode.PercentOutput, -Constants.kConveyorBackUpPercent);
                    backupStartTime = Timer.getFPGATimestamp();
                }
                if(Timer.getFPGATimestamp()-backupStartTime >= backupSeconds && !shooterChecked){
                    conveyorMaster.set(ControlMode.PercentOutput, 0.0);
                }

                if(driverControls.getBoolean(DriverControlsEnum.SHOOT)){
                    if(shooterChecked){
                        conveyorMaster.set(ControlMode.PercentOutput, Constants.kConveyorFeedPercent);
                        runHopper();
                        storageCount = 0;
                    } else {
                        stopHopper();
                        shooterChecked = Shooter.getInstance().nearTarget(true);
                    }
                } else {
                    kickerMotorMaster.set(ControlMode.PercentOutput, 0.0);
                    if(storageCount < 3 && exitProximitySensor.get()){
                        runHopper();
                        if(entranceEdge.update(!entranceProximitySensor.get())){
                            storageCount++;
                        }
                        if(!entranceProximitySensor.get()){
                            conveyorMaster.set(ControlMode.PercentOutput, Constants.kConveyorFeedPercent);
                        } else {
                            conveyorMaster.set(ControlMode.PercentOutput, 0.0);
                        }
                    } else {
                        conveyorMaster.set(ControlMode.PercentOutput, 0.0);
                        stopHopper();
                    }
                }
            }

            // if(shootDetector.update(driverControls.getBoolean(DriverControlsEnum.SHOOT))){
            //     setTowerPosition(getTowerPosInches()-Constants.kConveyorBackupDist);
            // }

            // //Starts feeding when shooter has achieved a high enough speed
            // if(driverControls.getBoolean(DriverControlsEnum.SHOOT)){
            //     if(!shooterChecked){
            //         shooterChecked = Shooter.getInstance().nearTarget(true);
            //     } else {
            //         setTowerIPS(Constants.kConveyorFeedIPS);
            //     }
            //     setKickerRPM(Shooter.getInstance().getTargetRPM()*Constants.kKickerProportion);
            // } else {
            //     setTowerIPS(0.0);
            //     setKickerRPM(0.0);
            // }
        } else {
            // //Favors velocity over position
            // if(SmartDashboard.getNumber("Conveyor/Debug/SetTowerIPS", 0) != 0){
            //     setTowerIPS(SmartDashboard.getNumber("Conveyor/Debug/SetTowerIPS", 0));
            // } else {
            //     setTowerPosition(SmartDashboard.getNumber("Conveyor/Debug/SetTowerPosInches", 0));
            // }

            leftHopperMotor.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Conveyor/Debug/SetLeftHopperPercent", 0));
            rightHopperMotor.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Conveyor/Debug/SetRightHopperPercent", 0));

            // setKickerRPM(SmartDashboard.getNumber("Conveyor/Debug/SetKickerRPM", 0));

            kickerMotorMaster.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Conveyor/Debug/SetKickerPercent", 0));

            conveyorMaster.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Conveyor/Debug/SetTowerPercent", 0));

            SmartDashboard.putBoolean("Conveyor/Debug/EntranceSensor", entranceProximitySensor.get());
            SmartDashboard.putBoolean("Conveyor/Debug/ExitSensor", exitProximitySensor.get());

            
            
            // //Outputting Sensor Data:
            // SmartDashboard.putNumber("Conveyor/Debug/SensedTowerIPS", getTowerIPS());
            // SmartDashboard.putNumber("Conveyor/Debug/SensedTowerPosInches", getTowerPosInches());
            // SmartDashboard.putNumber("Conveyor/Debug/KickerSensedRPM", getKickerSensedRPM());
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
        //setTowerIPS(0);
    }


    //================================
    //Tower Controls/Functions:
    //================================

    public void setTowerIPS(double ips){
        conveyorMaster.set(ControlMode.Velocity, ipsToEncoderUnitsPerFrameTower(ips));
    }

    public void setTowerPosition(double inches){
        //Moves the conveyor belt up or down some many inches. Positive moves it up
        conveyorMaster.set(ControlMode.MotionMagic, inchesToEncoderUnitsTower(inches) );
    }

    public void feed(){
        runHopper();
        conveyorMaster.set(ControlMode.PercentOutput, Constants.kConveyorFeedPercent);     

        //Old code that relied on encoders:
        //targetPosInches = getTowerPosInches() + (inchesPerBall*(double)numberOfBalls);
        //setTowerPosition(targetPosInches);
    }

    public void reverseTower(){
        conveyorMaster.set(ControlMode.PercentOutput, -Constants.kConveyorBackUpPercent);
    }

    public void stopTower(){
        conveyorMaster.set(ControlMode.PercentOutput, 0.0);
    }

    public void runKicker(){
        kickerMotorMaster.set(ControlMode.PercentOutput, Constants.kKickerShootPercent);
    }

    public void stopAll(){
        stopHopper();
        conveyorMaster.set(ControlMode.PercentOutput, 0.0);
        kickerMotorMaster.set(ControlMode.PercentOutput, 0.0);
    }




    public boolean nearTarget(){
        return (Math.abs(targetPosInches-getTowerPosInches())<=posToleranceInches);
    }

    public double getTowerPosInches(){
        return encoderUnitsToInchesTower(conveyorMaster.getSelectedSensorPosition());
    }

    public double getTowerIPS(){
        return encoderUnitsPerFrameToIPSTower(conveyorMaster.getSelectedSensorVelocity());
    }



    private static double encoderUnitsToInchesTower(double _units){
        return _units/kEncoderUnitsPerInchTower;
    }

    private static int inchesToEncoderUnitsTower(double _inches){
        return (int)(_inches*kEncoderUnitsPerInchTower);
    }

    private static int ipsToEncoderUnitsPerFrameTower(double _ips){
        return (int)((_ips*kEncoderUnitsPerInchTower)*kQuadEncoderStatusFramePeriod);
    }

    private static double encoderUnitsPerFrameToIPSTower(int _UPF){
        return (((double) _UPF/kEncoderUnitsPerInchTower)/kQuadEncoderStatusFramePeriod);
    }



    //=============================
    //Hopper Controls/Functions
    //=============================
    public void runHopper(){
        leftHopperMotor.set(ControlMode.PercentOutput, Constants.kLeftHopperPercent);
        rightHopperMotor.set(ControlMode.PercentOutput, Constants.kRightHopperPercent);
    }

    public void stopHopper(){
        leftHopperMotor.set(ControlMode.PercentOutput, 0.0);
        rightHopperMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
    public void reverseHopper(){
        leftHopperMotor.set(ControlMode.PercentOutput, -Constants.kHopperReversePercent);
        rightHopperMotor.set(ControlMode.PercentOutput, -Constants.kHopperReversePercent);
    }



    //==================
    //Kicker Controls
    //==================

    public void setKickerRPM(double RPM){
        //kickerMotor.set(ControlMode.Velocity, rpmToEncoderUnitsPerFrameKicker(RPM));
    }

    // public double getKickerSensedRPM(){
    //     return encoderUnitsPerFrameToRPMKicker(kickerMotor.getSelectedSensorVelocity());
    // }


    private static int rpmToEncoderUnitsPerFrameKicker(double _rpm){
        return (int)((_rpm*kEncoderUnitsPerRevKicker)*(kQuadEncoderStatusFramePeriod/60.0));
    }

    private static double encoderUnitsPerFrameToRPMKicker(int _UPF){
        return (((double) _UPF/kEncoderUnitsPerRevKicker)/kQuadEncoderStatusFramePeriod);
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