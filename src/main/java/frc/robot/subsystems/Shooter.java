package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

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
    public TalonSRX shooterMotor, hoodMotor, turretMotor;
    public VictorSPX shooterSlave;
    public Limelight camera = Limelight.getInstance();
    public double speed;

    //====================================================
    // Constants
    //====================================================
    
    public static final int kSlotIdxSpeed = 0;
    public static final int kSlotIdxPos   = 1;

    public final double kCalMaxEncoderPulsePer100ms = 33300;	// velocity at a max throttle (measured using Phoenix Tuner)
    public final double kCalMaxPercentOutput 		= 1.0;	// percent output of motor at above throttle (using Phoenix Tuner)

	public final double kKfShooterV = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public final double kKpShooterV = 0.03;	   
	public final double kKdShooterV = 2.0625;	// to resolve any overshoot, start at 10*Kp 
    public final double kKiShooterV = 0.0;  
    
    public final double kKfHoodPos = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public final double kKpHoodPos = 0.0;	   
	public final double kKdHoodPos = 0.0;	
    public final double kKiHoodPos = 0.0;
    
    public final double kKfTurretPos = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public final double kKpTurretPos = 0.0;	   
	public final double kKdTurretPos = 0.0;
	public final double kKiTurretPos = 0.0; 

	public static double kQuadEncoderCodesPerRev = 1024;
	public static double kQuadEncoderUnitsPerRev = 4*kQuadEncoderCodesPerRev;
    public static double kQuadEncoderStatusFramePeriod = 0.100; // 100 ms
    public static double kShooterGearRatio = 1.0/2.0;
    public static double kTurretGearRatio = 20.0/1.0;
    public static double kHoodGearRatio = 1.0/1.0;

    public final int kAllowableError = (int)rpmToEncoderUnitsPerFrame(10);

    public final int kPeakCurrentLimit = 50;
    public final int kPeakCurrentDuration = 200;
    public final int kContinuousCurrentLimit = 30;
    public final int kSliderMax = 200;

    public static double targetRPM = 0;
    public static double kRPMErrorShooting = 360.0, kRPMErrorStopping = 20.0;


    //Variables for Target Location and Shooter build =======================
    public static double targetHeight = 18.75; //Measured in inches
    public static double cameraHeight = 0;
    public static double shooterWheelRadius = 3.0; //Inches
    public static double cameraAngleDeg = 18; //From the horizontal
    public static double cameraAngleRad = Math.toRadians(cameraAngleDeg);
    public static Vector2d shooterPosFromCam = new Vector2d(-1.5, -1.5); //In inches. Front camera face is positive y. Measured from camera's center
    public static Vector2d shooterPosFromRobot = new Vector2d(-12, -12); // In inches. Front of robot is positive y. Measured from robot center

    public Vector2d targetPos;
    public static double targetSmoothing = (1.0/10.0);

    public Vector2d shooterVelocity;



    public static double maxRotationTrackingDeg = 360;
    public static double maxRotationDeg = 720;
    public static double maxRotationTrackingRad = Math.toRadians(maxRotationTrackingDeg);
    public static double maxRotationRad = Math.toRadians(maxRotationDeg);
    public static double searchingVelocityRPM = 0.5; 


    //Shooter Operational States
    public enum ShooterState {
        IDLING, WAITING, SEARCHING, TRACKING, READJUSTING, SHOOTING
    }
    ShooterState cState = ShooterState.SEARCHING;
    
    //Target Tracking Variables
    public int targetLostCount = 0; //This is used to provide a buffer before searching for the target
    public static int maxLostCountShooting = 5;
    public static int maxLostCountTracking = 10;
    public Vector2d lastTargetPos = null;

    public double targetAdjustRads = 0; //Used in the Readjusting state in order to return to the original absolute position
    public static double adjustmentToleranceDeg = 10; //Allowable error when readjusting
    public static double adjustmentToleranceRad = Math.toRadians(adjustmentToleranceDeg);


    // Distance vs. RPM & Hood Pos Table

    public double[][] dataTable = {
        {1,1,1},
        {1,1,1},
        {1,1,1},
        {1,1,1},
        {1,1,1},
        {1,1,1}
    };


    public Shooter() 
    {
        shooterMotor = new TalonSRX(Constants.kShooterTalonId);
        shooterSlave = new VictorSPX(Constants.kShooterSlaveId);

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
		shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
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
        shooterMotor.configAllowableClosedloopError(kSlotIdxSpeed, kAllowableError, Constants.kTalonTimeoutMs);
        
        // current limits
        shooterMotor.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        shooterMotor.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        shooterMotor.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        shooterMotor.enableCurrentLimit(true);

        // slave stuff
        shooterSlave.follow(shooterMotor);
        shooterSlave.setInverted(false);

        //=======================================
        //Hood Motor Config
        //=======================================
        // configure encoder
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
        hoodMotor.configAllowableClosedloopError(kSlotIdxPos, kAllowableError, Constants.kTalonTimeoutMs);


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
        turretMotor.configAllowableClosedloopError(kSlotIdxPos, kAllowableError, Constants.kTalonTimeoutMs);        
    }


    //Loop Functions
    @Override
    public void onStart() {
        stop();
        zeroSensors();
    }

    @Override
    public void onLoop() {
        if(!SmartDashboard.getBoolean("Shooter/Debug", false)){
            //Order of loop: 
            //First part: determining appropriate state:
            //  Check for target - determine if it has been lost - handle user input - determine if the turret is spun too far
            //Second part: react according to determined state
            //  Switch statement contains code necessary to each state 

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

            //Checking turret rotation to ensure wires don't get wrapped up:
            if(checkTurretOutOfBounds(cState) && cState != ShooterState.READJUSTING){
                cState = ShooterState.READJUSTING;
                targetAdjustRads = getTurretAbsoluteAngleRad();
            }
            

            //Reacting based on the determined cState:
            switch(cState){
                case IDLING:
                    //Hold everything in place
                    setShooterRPM(0);
                    setTurretDeg(0);
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
                    setTurretDeg(180-Pigeon.getInstance().getHeadingDeg()); //Always facing our port. Relies on proper initialization
                    break;

                case TRACKING:
                    setShooterRPM(0);
                    setHoodDeg(0);
                    setTurretDeg(targetDisplacement.angle()-(Math.PI/2.0)); //pi/2 subtracted to account for turret 0 is forward while target 0 is to the left
                    break;

                case READJUSTING:
                    setTurretDeg(Math.toDegrees(targetAdjustRads));
                    if(Math.abs(getTurretAngleRad()-targetAdjustRads) <= adjustmentToleranceRad){
                        cState = ShooterState.WAITING; //Send the shooter into waiting to see if target is detected
                    }
                    break;    

                case SHOOTING:
                    Vector2d ballVelocity = calcBallVelocity(targetDisplacement); //This is the target velocity of the ball immediately after leaving the robot
                    double shooterRPM = (2*ballVelocity.length()/shooterWheelRadius)*(30.0/Math.PI); //Determine RPM of shooter from new target velocity of ball
                    
                    double hoodDeg = calcHoodPosition(targetDisplacement.length());
                    double turretDeg = Math.toDegrees(ballVelocity.angle());

                    //Controlling subsystems:
                    setShooterRPM(shooterRPM);
                    setHoodDeg(hoodDeg);
                    setTurretDeg(turretDeg);
                    break;

                default:
                    break;
            }
        } else {
            setShooterRPM(SmartDashboard.getNumber("Shooter/RPM", 0));
            setHoodDeg(SmartDashboard.getNumber("Shooter/HoodDegree", 0));
            SmartDashboard.putNumber("Shooter/SensedRPM", encoderUnitsPerFrameToRPM(shooterMotor.getSelectedSensorVelocity()));
        }
    }

    @Override
    public void onStop() {
        stop();
    }

    public void stop()
    {
        setShooterRPM(0);
    }

    public void zeroSensors(){
        hoodMotor.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
        turretMotor.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
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
        double encoderUnits = hoodDegreesToEncoderUnits(degree);
        hoodMotor.set(ControlMode.Position, encoderUnits);
    }

    public void setTurretDeg(double degrees){
        //Relies on proper intialization of turret, facing forwards
        double radians = Math.toRadians(degrees);
        double cAbsAngle = getTurretAbsoluteAngleRad();
        double cAngle = getTurretAngleRad();

        double targetRads = getAngleError(radians, cAbsAngle) + cAngle;

        double encoderUnits = turretDegreesToEncoderUnits(Math.toDegrees(targetRads));
        turretMotor.set(ControlMode.MotionMagic, encoderUnits);
    }

    

    public double getSpeedError()
    {
        double sensorRPM = encoderUnitsPerFrameToRPM(shooterMotor.getSelectedSensorVelocity())*(1.0/kShooterGearRatio);
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
        if(camera.getIsTargetFound()){
            double targetY = (targetHeight-cameraHeight)/Math.tan(camera.getTargetVerticalAngleRad()+cameraAngleRad);
            double targetX = -targetY*Math.tan(camera.getTargetHorizontalAngleRad()); //Negative is to ensure that left of camera is positive from top-view
            Vector2d detectedTargetPos = new Vector2d(targetX, targetY);
            SmartDashboard.putNumber("Shooter/Targetx", targetX);
            SmartDashboard.putNumber("Shooter/Targety", targetY);
            SmartDashboard.putNumber("Shooter/TargetDist", detectedTargetPos.length());
            detectedTargetPos = detectedTargetPos.sub(shooterPosFromCam); //Map the detected vector onto the shooter's center
            detectedTargetPos = detectedTargetPos.rotate(getTurretAbsoluteAngleRad()); //This is used to rotate it back to the robot's perspective which is used to ground our measurements

            //Averaging:
            if(targetPos == null){
                //First time through
                targetPos = detectedTargetPos;
            } else {
                //Otherwise just keep averaging the position
                targetPos = targetPos.expAverage(detectedTargetPos, targetSmoothing); //Must update targetPos for next pass through the program
            }

            return targetPos;
        } else {
            return null; //In the event that the target can not be found
        }
    }

    public Vector2d getShooterVelocity(){
        double leftSpeed = DriveState.getInstance().getLeftSpeedInchesPerSec();
        double rightSpeed = DriveState.getInstance().getRightSpeedInchesPerSec();
        LinearAngularSpeed robotSpeed = Kinematics.forwardKinematics(leftSpeed, rightSpeed);
        if(robotSpeed.angularSpeed > Math.PI/6.0){
           //Only use if angular speed is of some significant amount
           double motionRadius = robotSpeed.linearSpeed/robotSpeed.angularSpeed;
           double shooterMotionRadius = lawOfCosines(motionRadius, shooterPosFromRobot.length(), shooterPosFromRobot.angle(new Vector2d(-1,0)));
           double shooterVelMagnitude = robotSpeed.angularSpeed*shooterMotionRadius;
           shooterVelocity = new Vector2d(Math.copySign(shooterVelMagnitude, robotSpeed.linearSpeed),0); //Shooter is considered '90' degrees as this is the direction of the robot
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
        return ((double)turretMotor.getSelectedSensorPosition()/(double)kQuadEncoderCodesPerRev)*(2.0*Math.PI)*(1.0/kTurretGearRatio);
    }

    public double getTurretAbsoluteAngleRad(){
        //Returns angle within pi to -pi. Positive is CCW
        double radians = getTurretAngleRad();
        double adjustedRad = radians % (2.0*Math.PI);
        adjustedRad = Math.abs(adjustedRad) < Math.PI ? adjustedRad : Math.copySign(Math.abs(adjustedRad)-(2.0*Math.PI),-adjustedRad);
        return adjustedRad;
    }

    public boolean checkTurretOutOfBounds(ShooterState state){
        switch(state){
            case SEARCHING:
            case TRACKING:
                return (Math.abs(getTurretAngleRad()) > maxRotationTrackingRad);

            case SHOOTING:
                return (Math.abs(getTurretAngleRad())> maxRotationRad);

            default:
                return true;
        }
    }



    public double getAngleError(double targetRad, double actualRad){
        //Positive output indicates CCW motion from actual to target
        actualRad = actualRad < 0 ? actualRad+(Math.PI*2.0):actualRad;
        targetRad = targetRad < 0 ? targetRad+(Math.PI*2.0):targetRad;

        actualRad -= targetRad; //Subtracting to make the target "0"

        actualRad = actualRad < 0 ? actualRad+(Math.PI*2.0):actualRad; //again readjusting to positive measures
        
        double distCW = actualRad-(Math.PI*2.0);
        double distCCW = actualRad;
        double output = Math.abs(distCW)<Math.abs(distCCW) ? distCW : distCCW;
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
        return rpmToEncoderUnitsPerFrame(_rpm*kShooterGearRatio);
    }

    // Turret conversions:
    public static int turretDegreesToEncoderUnits(double _degrees){
        return degreesToEncoderUnits(_degrees*kTurretGearRatio);
    }

    //Hood conversions:
    public static int hoodDegreesToEncoderUnits(double _degrees){
        return degreesToEncoderUnits(_degrees*kHoodGearRatio);
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
}