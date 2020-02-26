package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.command_status.DriveState;
import frc.robot.lib.joystick.DriverAxisEnum;
import frc.robot.lib.joystick.DriverControlsBase;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.sensors.Pigeon;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Kinematics;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.lib.util.Kinematics.LinearAngularSpeed;
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
    public TalonFX shooterMotor, shooterSlave;
    public TalonSRX turretMotor, hoodMotor;
    public Limelight camera = Limelight.getInstance();
    public Pigeon pigeon = (Pigeon)Pigeon.getInstance();
    public DigitalInput magnetSensor;
    public double speed;

    //====================================================
    // Constants
    //====================================================
    
    public static final int kSlotIdxSpeed = 0;
    public static final int kSlotIdxPos   = 1;

    public final double kCalMaxEncoderPulsePer100ms = 33300;	// velocity at a max throttle (measured using Phoenix Tuner)
    public final double kCalMaxPercentOutput 		= 1.0;	// percent output of motor at above throttle (using Phoenix Tuner)

	public final double kKfShooterV = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms; //0.03072071
	public final double kKpShooterV = 0.25;	   
	public final double kKdShooterV = 24.0;	// to resolve any overshoot, start at 10*Kp 
    public final double kKiShooterV = 0.0;  
    
    public final double kKfHoodPos = 0.0;
	public final double kKpHoodPos = 10.24;	   
	public final double kKdHoodPos = 32.0;	
    public final double kKiHoodPos = 0.0;
    
    public final double kKfTurretPos = 0.0;
	public final double kKpTurretPos = 5.0;   
	public final double kKdTurretPos = 150.0;
	public final double kKiTurretPos = 0.0; 

	public static double kQuadEncoderCodesPerRev = 1024;
	public static double kQuadEncoderUnitsPerRev = 4*kQuadEncoderCodesPerRev;
    public static double kQuadEncoderStatusFramePeriod = 0.100; // 100 ms
    public static double kEncoderUnitsPerRevShooter = 1540;
    public static double kEncoderUnitsPerRevHood = 104424;
    public static double kEncoderUnitsPerRevTurret = 49050;

    public final int kToleranceShooter = (int)rpmToEncoderUnitsPerFrame(10.0);
    public final int kToleranceTurret = turretDegreesToEncoderUnits(0.5);
    public final int kToleranceHood = hoodDegreesToEncoderUnits(1);

    public final int kPeakCurrentLimit = 50;
    public final int kPeakCurrentDuration = 200;
    public final int kContinuousCurrentLimit = 30;
    public final int kSliderMax = 200;

    public static double targetRPM = 0;
    public static double kRPMErrorShooting = 360.0, kRPMErrorStopping = 20.0;


    //Variables for Target Location and Shooter build =======================
    public static final double targetHeight = 99.0; //Measured in inches
    public static final double cameraHeight = 41.0;
    public static final double shooterWheelRadius = 3.0; //Inches
    public static final double cameraAngleDeg = 25.0; //From the horizontal
    public static final double cameraAngleRad = Math.toRadians(cameraAngleDeg);
    public static final Vector2d shooterPosFromCam = new Vector2d(0, 0); //In inches. Front camera face is positive y. Measured from camera's center
    public static final Vector2d shooterPosFromRobot = new Vector2d(-12, -12); // In inches. Front of robot is positive y. Measured from robot center

    public Vector2d targetPos;
    public static final double targetSmoothing = (1.0/6.0);

    public Vector2d shooterVelocity;


    public static final double[] searchingIntervalDeg = {-90, 90};
    public static final double[] trackingIntervalDeg = {-170,170};
    public static final double[] shootingIntervalDeg = {-170,170};

    public static final double trackingRangeDeg = Math.abs(trackingIntervalDeg[1]-trackingIntervalDeg[0]);
    public static final double searchingRangeDeg = Math.abs(searchingIntervalDeg[1]-searchingIntervalDeg[0]);
    public static final double shootingRangeDeg = Math.abs(shootingIntervalDeg[1]-shootingIntervalDeg[0]);
    public static final double searchingRPM = 0.25; 

    public static final double maxHoodDeg = 47.00; //Tested


    //Shooter Operational States
    public enum ShooterState {
        IDLING, WAITING, SEARCHING, TRACKING, READJUSTING, SHOOTING
    }
    ShooterState cState = ShooterState.SEARCHING;
    
    //Target Tracking Variables
    public int targetLostCount = 0; //This is used to provide a buffer before searching for the target
    public static final int maxLostCountShooting = 5;
    public static final int maxLostCountTracking = 10;
    public Vector2d lastTargetPos = null;

    public double targetAdjustRads = 0; //Used in the Readjusting state in order to return to the original absolute position
    public static final double adjustmentToleranceDeg = 10; //Allowable error when readjusting
    public static final double adjustmentToleranceRad = Math.toRadians(adjustmentToleranceDeg);
    public boolean readjustmentEnabled = true;

    public double homeAngleRadTurret = 0; //Used in tracking where zero is physically
    public double homeAngleDegHood = 0;

    public double sweepStartTime = 0;
    public double sweepPhaseAngleDeg = 0;
    public static final double sweepRangeDeg = 90;
    public double[] sweepIntervalDeg = {-sweepRangeDeg/2.0,sweepRangeDeg/2.0};


    // Distance vs. RPM & Hood Pos Table

    public double[][] dataTable = {
        {24,3600,0},
        {49,2750,22},
        {105,3000,35},
        {189,4000,45},
        {265,4750,47},
    };

    //=================================
    //Test Mode Calibration Variables:
    private boolean turretCalibrated = false, hoodCalibrated = false, allCalibrated = false;
    private RisingEdgeDetector turretMagnetDetector = new RisingEdgeDetector();
    private RisingEdgeDetector hoodCurrentDetector = new RisingEdgeDetector();
    private static final double hoodThresholdCurrent = 100000;
    private static final double turretMagnetAngleDeg = 1000; //This is the what turret would output when lined up with magnet and calibrated properly





    public Shooter() 
    {
        shooterMotor = new TalonFX(Constants.kShooterTalonId);
        shooterSlave = new TalonFX(Constants.kShooterSlaveId);

        hoodMotor = new TalonSRX(Constants.kShooterHoodID);
        turretMotor = new TalonSRX(Constants.kShooterTurretID);

        // Factory default hardware to prevent unexpected behavior
        shooterMotor.configFactoryDefault();
        hoodMotor.configFactoryDefault();
        turretMotor.configFactoryDefault();

        //=======================================
        //Shooter Motor Config
        //=======================================
		// configure encoder
		shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 1, Constants.kTalonTimeoutMs);
        shooterMotor.setSensorPhase(false); // set so that positive motor input results in positive change in sensor value
        shooterMotor.setInverted(true);   // set to have green LEDs when driving forward
		
		// set relevant frame periods to be at least as fast as periodic rate
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        shooterMotor.selectProfileSlot(kSlotIdxSpeed, Constants.kTalonPidIdx); 
        shooterMotor.config_kF(kSlotIdxSpeed, kKfShooterV, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kP(kSlotIdxSpeed, kKpShooterV, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kI(kSlotIdxSpeed, kKiShooterV, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kD(kSlotIdxSpeed, kKdShooterV, Constants.kTalonTimeoutMs);
        shooterMotor.configAllowableClosedloopError(kSlotIdxSpeed, kToleranceShooter, Constants.kTalonTimeoutMs);
        shooterMotor.configClosedloopRamp(2, Constants.kTalonTimeoutMs);
        
        // current limits
        // shooterMotor.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        // shooterMotor.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        // shooterMotor.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        // shooterMotor.enableCurrentLimit(true);

        // slave stuff
        shooterSlave.follow(shooterMotor);
        shooterSlave.setInverted(false);

        // =======================================
        // Hood Motor Config
        // =======================================
        //configure encoder
		hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		hoodMotor.setSensorPhase(false); // set so that positive motor input results in positive change in sensor value
        hoodMotor.setInverted(true);   // set to have green LEDs when driving forward
        hoodMotor.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		
		// set relevant frame periods to be at least as fast as periodic rate
		hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure position loop PID 
        hoodMotor.selectProfileSlot(kSlotIdxPos, Constants.kTalonPidIdx); 
        hoodMotor.config_kF(kSlotIdxPos, kKfHoodPos, Constants.kTalonTimeoutMs); 
        hoodMotor.config_kP(kSlotIdxPos, kKpHoodPos, Constants.kTalonTimeoutMs); 
        hoodMotor.config_kI(kSlotIdxPos, kKiHoodPos, Constants.kTalonTimeoutMs); 
        hoodMotor.config_kD(kSlotIdxPos, kKdHoodPos, Constants.kTalonTimeoutMs);
        hoodMotor.configAllowableClosedloopError(kSlotIdxPos, kToleranceHood, Constants.kTalonTimeoutMs);


        //=======================================
        //Turret Motor Config
        //=======================================
        // configure encoder
		turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		turretMotor.setSensorPhase(false); // set so that positive motor input results in positive change in sensor value
        turretMotor.setInverted(true);   // set to have green LEDs when driving forward
        turretMotor.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		
		// set relevant frame periods to be at least as fast as periodic rate
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure position loop PID 
        turretMotor.selectProfileSlot(kSlotIdxPos, Constants.kTalonPidIdx); 
        turretMotor.config_kF(kSlotIdxPos, kKfTurretPos, Constants.kTalonTimeoutMs); 
        turretMotor.config_kP(kSlotIdxPos, kKpTurretPos, Constants.kTalonTimeoutMs); 
        turretMotor.config_kI(kSlotIdxPos, kKiTurretPos, Constants.kTalonTimeoutMs); 
        turretMotor.config_kD(kSlotIdxPos, kKdTurretPos, Constants.kTalonTimeoutMs);
        turretMotor.configAllowableClosedloopError(kSlotIdxPos, kToleranceTurret, Constants.kTalonTimeoutMs);

        // turretMotor.configMotionAcceleration(rpmToEncoderUnitsPerFrame(10));
        // turretMotor.configMotionCruiseVelocity(rpmToEncoderUnitsPerFrame(2));

        magnetSensor = new DigitalInput(Constants.kTurretHallEffect);
    }


    //Loop Functions
    @Override
    public void onStart() {
        //stop();
    }

    @Override
    public void onLoop() {
        if(!SmartDashboard.getBoolean("Shooter/Debug", false)){
            //Checking the target status and determining its relative displacement:
            Vector2d targetDisplacement;
            if(camera.getIsTargetFound()){
                targetLostCount = 0;
                targetDisplacement = getTargetDisplacement();
                if(cState != ShooterState.SHOOTING){
                    cState = ShooterState.TRACKING; //Jump to the tracking status in order to keep it in view
                }
            } else if(cState == ShooterState.READJUSTING){
                //Don't add to the counter, give the turret time to readjust
                targetDisplacement = lastTargetPos; 
            } else {
                targetLostCount++;
                targetDisplacement = lastTargetPos;
            }
            lastTargetPos = targetDisplacement;

            //Determining if a the state should be swapped to handle target loss:
            if(targetLostCount >= maxLostCountTracking){
                cState = ShooterState.SEARCHING; //Begin actively looking for the target
            } else if(targetLostCount >= maxLostCountShooting) {
                cState = ShooterState.WAITING; //Stop shooting and wait to see if target comes back
            }

            //Taking in user input and adjusting state accordingly
            DriverControlsBase driverControls = SelectedDriverControls.getInstance().get();
            if(driverControls.getBoolean(DriverControlsEnum.SHOOT) && cState == ShooterState.TRACKING){
                cState = ShooterState.SHOOTING;
            }


            //Checking if we can even readjust
            switch(cState){
                case SEARCHING:
                    readjustmentEnabled = searchingRangeDeg > 360.0;
                case TRACKING:
                    readjustmentEnabled = trackingRangeDeg > 360.0;
                case SHOOTING:
                    readjustmentEnabled = shootingRangeDeg > 360.0;
                default:
                    readjustmentEnabled = false;
            }
            //Checking turret rotation to ensure wires don't get wrapped up:
            if(readjustmentEnabled){
                if(checkTurretOutOfBounds(cState) && cState != ShooterState.READJUSTING){
                    cState = ShooterState.READJUSTING;
                    targetAdjustRads = getTurretAbsoluteAngleRad();
                }
            } else {
                if(checkTurretOutOfBounds(cState)){
                    cState = ShooterState.WAITING;
                    setTurretDeg(limitTurretInputDeg(Math.toDegrees(getTurretAngleRad()))); //Send it to the closest bound
                }
            }
            

            //Reacting based on the determined cState:
            switch(cState){
                case IDLING:
                    //Hold everything in place
                    setShooterRPM(0);
                    setTurretAbsDeg(0);
                    setHoodDeg(0);
                    break;

                case WAITING:
                    //Nothing really occurs in this state and usually doesn't last long
                    //This is used as a back up while shooting if the target is temporarily lost
                    break;

                case SEARCHING:
                    //Look around
                    setShooterRPM(0);
                    setHoodDeg(0);
                    double cHeading = pigeon.getHeadingDeg();
                    setTurretAbsDeg(Math.copySign(180-Math.abs(cHeading), cHeading)); //Always facing our port. Relies on proper initialization
                    break;

                case TRACKING:
                    setShooterRPM(0);
                    setHoodDeg(0);
                    setTurretAbsDeg(targetDisplacement.angle());
                    break;

                case READJUSTING:
                    setTurretDeg(Math.toDegrees(targetAdjustRads));
                    if(Math.abs(getTurretAngleRad()-targetAdjustRads) <= adjustmentToleranceRad){
                        cState = ShooterState.WAITING; //Send the shooter into waiting to see if target is detected
                    }
                    break;    

                case SHOOTING:
                    //Code that actually works:
                    int keyL = getLinear(targetDisplacement.length(), dataTable);
                    double nominalSpeed = handleLinear(targetDisplacement.length(), dataTable[keyL][0], dataTable[keyL+1][0], dataTable[keyL][1], dataTable[keyL+1][1]);

                    //double ballVelocity = getTargetBallVelMag(targetDisplacement.length());
                    double hoodDeg = calcHoodPosition(targetDisplacement.length());

                    //Controlling subsystems:
                    setShooterRPM(nominalSpeed);
                    setHoodDeg(hoodDeg);

                    double error = getTurretAbsoluteAngleRad()-targetDisplacement.angle();
                    if(Math.abs(error)>=Math.toRadians(0.5)){
                        setTurretAbsDeg(Math.toDegrees(targetDisplacement.angle()));
                    }


                    //Fancy Code:

                    // Vector2d ballVelocity = calcBallVelocity(targetDisplacement); //This is the target velocity of the ball immediately after leaving the robot
                    // double shooterRPM = (2*ballVelocity.length()/shooterWheelRadius)*(30.0/Math.PI); //Determine RPM of shooter from new target velocity of ball
                    
                    // double hoodDeg = calcHoodPosition(targetDisplacement.length());
                    // double turretDeg = Math.toDegrees(ballVelocity.angle());

                    // //Controlling subsystems:
                    // setShooterRPM(shooterRPM);
                    // setHoodDeg(hoodDeg);
                    // setTurretAbsDeg(turretDeg);
                    break;

                default:
                    break;
            }
        } else {
            //Debugging Code:

            //Manual controls:
            setShooterRPM(SmartDashboard.getNumber("Shooter/Debug/SetShooterRPM", 0));

            Vector2d targetDisplacement = getTargetDisplacement();
            targetDisplacement = targetDisplacement != null ? targetDisplacement : new Vector2d(1,0); //Use of a unit vector in null state
            double error = getTurretAbsoluteAngleRad()-targetDisplacement.angle();
            if(Math.abs(error)>=Math.toRadians(0.5)){
                setTurretAbsDeg(Math.toDegrees(targetDisplacement.angle()));
            }
            setTurretDeg(SmartDashboard.getNumber("Shooter/Debug/SetTurretDeg", 0));

            setHoodDeg(SmartDashboard.getNumber("Shooter/Debug/SetHoodDeg", 0));


            //For testing auto-shooting:

            // Vector2d targetDisplacement = getTargetDisplacement();
            // if(targetDisplacement != null){

            //     int keyL = getLinear(targetDisplacement.length(), dataTable);
            //     double nominalSpeed = handleLinear(targetDisplacement.length(), dataTable[keyL][0], dataTable[keyL+1][0], dataTable[keyL][1], dataTable[keyL+1][1]);

            //     //double ballVelocity = getTargetBallVelMag(targetDisplacement.length());
            //     double hoodDeg = calcHoodPosition(targetDisplacement.length());

            //     //Controlling subsystems:
            //     setShooterRPM(nominalSpeed);
            //     setHoodDeg(hoodDeg);

            //     double error = getTurretAbsoluteAngleRad()-targetDisplacement.angle();
            //     if(Math.abs(error)>=Math.toRadians(0.5)){
            //         setTurretAbsDeg(Math.toDegrees(targetDisplacement.angle()));
            //     }
            // }


            //Feeding Smart Dashboard Info
            SmartDashboard.putNumber("Shooter/SensedRPM", getShooterSensedRPM());
            SmartDashboard.putNumber("Shooter/SensedHoodDeg", getHoodDeg());
            SmartDashboard.putNumber("Shooter/TurretRad", getTurretAngleRad());
            SmartDashboard.putNumber("Shooter/TurretRadAbs", getTurretAbsoluteAngleRad());
        }
    }

    @Override
    public void onStop() {
        //stop();
    }

    public void stop()
    {
        setShooterRPM(0);
    }

    public void zeroSensors(){
        // hoodMotor.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
        // turretMotor.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
    }



    public void setShooterRPM(double rpm)
    {
        targetRPM = rpm;
        double encoderSpeed = shooterRPMToEncoderUnitsPerFrame(targetRPM);
        shooterMotor.set(ControlMode.Velocity, encoderSpeed);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/EncoderSpeed", encoderSpeed);
    }

    public void setHoodDeg(double degree){
        //Relies on hood to be initialized properly
        degree = degree < 0 ? degree = 0 : degree>maxHoodDeg ? degree = maxHoodDeg : degree;
        degree += homeAngleDegHood;
        double encoderUnits = hoodDegreesToEncoderUnits(degree);
        hoodMotor.set(ControlMode.Position, encoderUnits);
    }

    public void setTurretAbsDeg(double degrees){
        //Relies on proper intialization of turret, facing forwards
        double radians = Math.toRadians(degrees);
        double cAbsAngle = getTurretAbsoluteAngleRad();
        double cAngle = getTurretAngleRad();

        double adjustment = getAngleError(radians, cAbsAngle, true);
        double targetRads = cAngle + adjustment;

        double targetDegs = Math.toDegrees(targetRads);

        //If the target goes out of bounds and the turret can not readjust:
        if(!readjustmentEnabled && targetDegs != limitTurretInputDeg(targetDegs)){
            //Determine if the turret can reach the target going the opposite direction
            if(degrees == limitTurretInputDeg(degrees)){
                //This indicates that the target does fall within the bounds and can be reached going in the opposite direction
                targetRads += Math.copySign(Math.PI*2.0, -adjustment); //Rotate the target one rev in the opposite direction
            } else {
                //This indicates that the target falls outside of both bounds
                targetRads = Math.toRadians(getClosestBound(targetDegs)); //Send it to the closest bound
            }
        }
        setTurretDeg(Math.toDegrees(targetRads));
    }

    public void setTurretDeg(double degree){
        if(!readjustmentEnabled){
            degree = limitTurretInputDeg(degree);
        }
        degree += Math.toDegrees(homeAngleRadTurret); //Adjusting for home position
        double encoderUnits = turretDegreesToEncoderUnits(degree);
        turretMotor.set(ControlMode.Position, encoderUnits);
    }


    
    public double getSpeedError()
    {
        double sensorRPM = getShooterSensedRPM();
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

    public double getTargetRPM(){
        return targetRPM;
    }



    //=======================================================
    //Primary Functions for Shooting Calculations
    //=======================================================


    public double calcHoodPosition(double targetDistance){
        //Returns hood position in degrees
        int keyL = getLinear(targetDistance, dataTable);
        double hoodPosition = handleLinear(targetDistance, dataTable[keyL][0], dataTable[keyL+1][0], dataTable[keyL][2], dataTable[keyL+1][2]);
        return hoodPosition;
    }


    public Vector2d calcBallVelocity(Vector2d targetDisplacement){
        double targetVelocityMag = getTargetBallVelMag(targetDisplacement.length());
        Vector2d targetVelocity = new Vector2d(0, targetVelocityMag); //Used to maintain magnitude
        targetVelocity.rotate(targetDisplacement.angle()); //Rotating it back to the correct rotation 

        Vector2d shooterVelocity = getShooterVelocity(); //Get the velocity of the shooter on the robot

        Vector2d targetBallVelocity = targetVelocity.sub(shooterVelocity); //Determine the necessary velocity of the ball being shot

        return targetBallVelocity;
    }


    public Vector2d getTargetDisplacement(){
        //Target Displacement angle is on the interval [-pi,pi]
        if(camera.getIsTargetFound()){
            double targetY = (targetHeight-cameraHeight)/Math.tan(camera.getTargetVerticalAngleRad()+cameraAngleRad);
            double targetX = targetY*Math.tan(-camera.getTargetHorizontalAngleRad()); //Negative is to ensure that left of camera is positive from top-view
            Vector2d detectedTargetPos = new Vector2d(targetX, targetY);
            SmartDashboard.putNumber("Shooter/Targetx", targetX);
            SmartDashboard.putNumber("Shooter/Targety", targetY);
            SmartDashboard.putNumber("Shooter/TargetDist", detectedTargetPos.length());
            detectedTargetPos = detectedTargetPos.rotate(-Math.PI/2.0); //Forward is considered at 90 degrees when sensed but should be 0.
            detectedTargetPos = detectedTargetPos.sub(shooterPosFromCam); //Map the detected vector onto the shooter's center
            detectedTargetPos = detectedTargetPos.rotate(getTurretAbsoluteAngleRad()); //This is used to rotate it back to the robot's perspective which is used to ground our measurements

            //Averaging:
            if(targetPos == null){
                //First time through
                targetPos = detectedTargetPos;
            } else {
                targetPos = targetPos.expAverage(detectedTargetPos, targetSmoothing);
            }
            SmartDashboard.putNumber("Shooter/Angle", targetPos.angle());
            return targetPos;
        } 
        else {
            return null; //In the event that the target can not be found
        }
    }

    public Vector2d getShooterVelocity(){
        double leftSpeed = DriveState.getInstance().getLeftSpeedInchesPerSec();
        double rightSpeed = DriveState.getInstance().getRightSpeedInchesPerSec();
        LinearAngularSpeed robotSpeed = Kinematics.forwardKinematics(leftSpeed, rightSpeed);
        if(robotSpeed.angularSpeed > Math.PI/6.0){
            //Only use if angular speed is of some significant amount
            double motionRadius = Math.abs(robotSpeed.linearSpeed/robotSpeed.angularSpeed);
            Vector2d robotRadius = robotSpeed.angularSpeed > 0 ? new Vector2d(0,motionRadius) : new Vector2d(0,-motionRadius);
            Vector2d shooterRadius = shooterPosFromRobot.sub(robotRadius);
            double shooterVelMag = Math.abs(shooterRadius.length()*robotSpeed.angularSpeed);
            Vector2d shooterVelocity = new Vector2d(shooterVelMag,0);
            double shooterVelAng = robotRadius.angle()+shooterRadius.angle();
            shooterVelocity = robotSpeed.linearSpeed>0 ? (shooterVelocity.rotate(shooterVelAng)) : (shooterVelocity.rotate(shooterVelAng+Math.PI));
        } else {
            shooterVelocity = new Vector2d(robotSpeed.linearSpeed, 0);
        }
        return shooterVelocity;
    }

    public double getTargetBallVelMag(double distance){
        SelectedDriverControls driverControls = SelectedDriverControls.getInstance();

        int keyL = getLinear(distance, dataTable);
        double shooterCorrection = -driverControls.getAxis(DriverAxisEnum.SHOOTER_SPEED_CORRECTION)*kSliderMax;
        double nominalSpeed = handleLinear(distance, dataTable[keyL][0], dataTable[keyL+1][0], dataTable[keyL][1], dataTable[keyL+1][1]);
        speed = nominalSpeed + shooterCorrection;
        return (0.5*speed*shooterWheelRadius); //Determine the balls' velocity
    }

    public double getTargetShooterVelocity(double distance){
        //This is more useful for autonomous
        return (getTargetBallVelMag(distance)/(0.5*shooterWheelRadius))*(30.0/Math.PI);
    }

    public double getTurretAngleRad(){
        //Turret should be zeroed when facing front of robot
        double measuredRad = (((double)turretMotor.getSelectedSensorPosition()/kEncoderUnitsPerRevTurret)*(Math.PI*2.0));
        double adjustedRad = measuredRad - homeAngleRadTurret; //Adjusting rads to physical position
        return adjustedRad;
    }

    public double getTurretAbsoluteAngleRad(){
        //Returns angle within pi to -pi. Positive is CCW
        double radians = getTurretAngleRad();
        double adjustedRad = radians % (2.0*Math.PI);
        adjustedRad = Math.abs(adjustedRad) < Math.PI ? adjustedRad : Math.copySign(Math.abs(adjustedRad)-(2.0*Math.PI),-adjustedRad);
        return adjustedRad;
    }

    public boolean checkTurretOutOfBounds(ShooterState state){
        double cAngleDeg = Math.toDegrees(getTurretAngleRad());
        switch(state){
            case SEARCHING:
                return cAngleDeg > searchingIntervalDeg[1] || cAngleDeg < searchingIntervalDeg[0];
            case TRACKING:
                return cAngleDeg > trackingIntervalDeg[1] || cAngleDeg < trackingIntervalDeg[0];
            case SHOOTING:
                return cAngleDeg > shootingIntervalDeg[1] || cAngleDeg < shootingIntervalDeg[0];
            default:
                return false;
        }
    }

    public double limitTurretInputDeg(double degrees){
        switch(cState){
            case SEARCHING:
                return limitValue(degrees, searchingIntervalDeg[0], searchingIntervalDeg[1]);
            case TRACKING:
                return limitValue(degrees, trackingIntervalDeg[0], trackingIntervalDeg[1]);
            case SHOOTING:
                return limitValue(degrees, searchingIntervalDeg[0], searchingIntervalDeg[1]);
            default:
                return 0;
        }
    }

    private double getClosestBound(double targetDegree){
        double targetRad = Math.toRadians(targetDegree);
        switch(cState){
            case SEARCHING:
                return Math.abs(getAngleError(targetRad, Math.toRadians(searchingIntervalDeg[0]), false)) < Math.abs(getAngleError(targetRad, Math.toRadians(searchingIntervalDeg[1]), false)) ? searchingIntervalDeg[0] : searchingIntervalDeg[1];
            case TRACKING:
                return Math.abs(getAngleError(targetRad, Math.toRadians(trackingIntervalDeg[0]), false)) < Math.abs(getAngleError(targetRad, Math.toRadians(trackingIntervalDeg[1]), false)) ? trackingIntervalDeg[0] : trackingIntervalDeg[1];
            case SHOOTING:
                return Math.abs(getAngleError(targetRad, Math.toRadians(shootingIntervalDeg[0]), false)) < Math.abs(getAngleError(targetRad, Math.toRadians(shootingIntervalDeg[1]), false)) ? shootingIntervalDeg[0] : shootingIntervalDeg[1];
            default:
                return 0;
        }
    }

    public double getShooterSensedRPM(){
        return (((double)shooterMotor.getSelectedSensorVelocity()/kEncoderUnitsPerRevShooter)*(60.0/kQuadEncoderStatusFramePeriod));
    }

    public double getHoodDeg(){
        return (((double)hoodMotor.getSelectedSensorPosition()/kEncoderUnitsPerRevHood)*(360.0));
    }




    private double limitValue(double value, double min, double max){
        return value > max ? max : value < min ? min : value;
    }

    public double getAngleError(double targetRad, double actualRad, boolean absoluteRad){
        //Positive output indicates CCW motion from actual to target

        // actualRad = actualRad < 0 ? actualRad+(Math.PI*2.0):actualRad;
        // targetRad = targetRad < 0 ? targetRad+(Math.PI*2.0):targetRad;

        //Getting positive values:
        while(actualRad < 0 || targetRad < 0){
            actualRad += Math.PI*2.0;
            targetRad += Math.PI*2.0;
        }

        actualRad -= targetRad; //Subtracting to make the target "0"

        double output;
        if(absoluteRad){
            while(actualRad < 0 || actualRad > Math.PI*2.0){
                //Adjusting into bounds
                actualRad = actualRad < 0 ? actualRad+Math.PI*2.0 : actualRad > Math.PI*2.0 ? actualRad-Math.PI*2.0 : actualRad;
            }
            
            double distCW = -actualRad;
            double distCCW = (Math.PI*2.0)-actualRad;
            output = Math.abs(distCW)<Math.abs(distCCW) ? distCW : distCCW;
        } else {
            output = actualRad; //Raw difference
        }
        return output;
    }

    public double lawOfCosines(double _leg1, double _leg2, double angleRad){
        //Very original function
        return Math.sqrt(Math.pow(_leg1, 2) + Math.pow(_leg2, 2) - 2*_leg1*_leg2*Math.cos(angleRad));
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


    //Shooter conversions:
    public static int shooterRPMToEncoderUnitsPerFrame(double _rpm){
        return (int)((_rpm*kEncoderUnitsPerRevShooter)*(kQuadEncoderStatusFramePeriod/60.0));
    }

    // Turret conversions:
    public static int turretDegreesToEncoderUnits(double _degrees){
        return (int)((_degrees/360.0)*kEncoderUnitsPerRevTurret);
    }

    //Hood conversions:
    public static int hoodDegreesToEncoderUnits(double _degrees){
        return (int)((_degrees/360.0)*kEncoderUnitsPerRevHood);
    }


    // Talon SRX reports position in rotations while in closed-loop Position mode
	public static double encoderUnitsToRevolutions(int _encoderPosition) {	return (double)_encoderPosition / (double)kQuadEncoderUnitsPerRev; }
    public static int revolutionsToEncoderUnits(double _rev) { return (int)(_rev * kQuadEncoderUnitsPerRev); }
    public static int degreesToEncoderUnits(double _deg) {return (int)((_deg/360.0)*kQuadEncoderCodesPerRev);}

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	public static double encoderUnitsPerFrameToRPM(int _encoderEdgesPerFrame) { return encoderUnitsToRevolutions(_encoderEdgesPerFrame) * 60.0 / kQuadEncoderStatusFramePeriod; }
	public static int rpmToEncoderUnitsPerFrame(double _rpm) { return (int)(revolutionsToEncoderUnits(_rpm) / 60.0 * kQuadEncoderStatusFramePeriod); }




    
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




    public void calibrate(){
        if(!turretCalibrated){
            turretMotor.set(ControlMode.PercentOutput, -0.25); //Turn right
            if(turretMagnetDetector.update(magnetSensor.get())){
                turretMotor.set(ControlMode.PercentOutput, 0.0);
                turretMotor.setSelectedSensorPosition(turretDegreesToEncoderUnits(turretMagnetAngleDeg), Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
                //homeAngleRadTurret = getTurretAngleRad()-Math.toRadians(turretMagnetAngleDeg);
                turretCalibrated = true;
            }
        }
        if(!hoodCalibrated){
            hoodMotor.set(ControlMode.PercentOutput, -.25); //Move down
            if(hoodCurrentDetector.update(hoodMotor.getStatorCurrent() >= hoodThresholdCurrent)){
                hoodMotor.set(ControlMode.PercentOutput, 0.0);
                hoodMotor.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
                //homeAngleDegHood = getHoodDeg();
                hoodCalibrated = true;
            }
        }
        
        if(hoodCalibrated && turretCalibrated && !allCalibrated){
            allCalibrated = true;
            System.out.println("Turret and hood calibrated!");
        }
    }
}