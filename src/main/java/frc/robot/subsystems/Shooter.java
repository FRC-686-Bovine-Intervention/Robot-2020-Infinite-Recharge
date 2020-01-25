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
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.sensors.Limelight;
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

    public final int kAllowableError = (int)rpmToEncoderUnitsPerFrame(10);

    public final int kPeakCurrentLimit = 50;
    public final int kPeakCurrentDuration = 200;
    public final int kContinuousCurrentLimit = 30;
    public final int kSliderMax = 200;

    public double kGoalHeight = 76.75;
    public double kCameraHeight = 23;
    public double kCameraAngle = 37;
    public double kFrontToCameraDist = 27.5;

    public static double targetRPM = 0;
    public static double kRPMErrorShooting = 360.0, kRPMErrorStopping = 20.0;
 
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
            setShooterRPM(SmartDashboard.getNumber("Shooter/RPM", 0));
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
    


    // Talon SRX reports position in rotations while in closed-loop Position mode
	public static double encoderUnitsToRevolutions(int _encoderPosition) {	return (double)_encoderPosition / (double)kQuadEncoderUnitsPerRev; }
    public static int revolutionsToEncoderUnits(double _rev) { return (int)(_rev * kQuadEncoderUnitsPerRev); }
    public static int degreesToEncoderUnits(double _deg) {return (int)((_deg/360.0)*kQuadEncoderCodesPerRev);}

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	public static double encoderUnitsPerFrameToRPM(int _encoderEdgesPerFrame) { return encoderUnitsToRevolutions(_encoderEdgesPerFrame) * 60.0 / kQuadEncoderStatusFramePeriod; }
	public static int rpmToEncoderUnitsPerFrame(double _rpm) { return (int)(revolutionsToEncoderUnits(_rpm) / 60.0 * kQuadEncoderStatusFramePeriod); }




    public void setShooterRPM(double rpm)
    {
        targetRPM = rpm;
        double encoderSpeed = rpmToEncoderUnitsPerFrame(targetRPM);
        shooterMotor.set(ControlMode.Velocity, encoderSpeed);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/EncoderSpeed", encoderSpeed);
    }

    public void setHoodDeg(double degree){
        //Relies on hood to be initialized properly
        double encoderTicks = degreesToEncoderUnits(degree);
        hoodMotor.set(ControlMode.Position, encoderTicks);
    }

    public void setTurretDeg(double degrees){
        double encoderUnits = degreesToEncoderUnits(degrees);
        hoodMotor.set(ControlMode.Position, encoderUnits);
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

    public void setShooter()
    {
        SelectedDriverControls driverControls = SelectedDriverControls.getInstance();


        double distance = getDistFromTarget(camera.getTargetVerticalAngleRad())-kFrontToCameraDist;
        int keyL = getLinear(distance, dataTable);
        double shooterCorrection = -driverControls.getAxis(DriverAxisEnum.SHOOTER_SPEED_CORRECTION)*kSliderMax;
        double nominalSpeed = handleLinear(distance, dataTable[keyL][0], dataTable[keyL+1][0], dataTable[keyL][1], dataTable[keyL+1][1]);
        speed = nominalSpeed + shooterCorrection;
        double hoodPosition = handleLinear(distance, dataTable[keyL][0], dataTable[keyL+1][0], dataTable[keyL][2], dataTable[keyL+1][2]);
        
        setHoodDeg(hoodPosition);
        setShooterRPM(speed);
    }





    public double getDistFromTarget(double angleRad){
        return (kGoalHeight-kCameraHeight)/(Math.tan(angleRad+(kCameraAngle * Vector2d.degreesToRadians)));
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











    //Cool Shooter Stuff by Roame=======================================

    public static double targetHeight = 72.5; //Measured in inches
    public static double shooterWheelRadius = 3.0; //Inches
    public static double cameraAngle = 45;
    public static double cameraAngleRad = Math.toRadians(cameraAngle);
    public static Vector2d shooterPosFromCam = new Vector2d(-1.5, -1.5); //In inches. Front camera face is positive y. Measured from camera's center
    public static Vector2d shooterPosFromRobot = new Vector2d(-12, -12); // In inches. Front of robot is positive y. Measured from robot center
    
    public Vector2d targetPos;
    public static double targetSmoothing = (1.0/20.0);

    public static Vector2d shooterVelocity;


    public void runShooter(){
        Vector2d ballVelocity = calcBallVelocity(); //This is the target velocity of the ball immediately after leaving the robot
        double shooterRPM = (2*ballVelocity.length()/shooterWheelRadius)*(30.0/Math.PI); //Determine RPM of shooter from new target velocity of ball
        double hoodDeg = calcHoodPosition();
        double turretDeg = Math.toDegrees(ballVelocity.angle());

        //Controlling subsystems:
        setShooterRPM(shooterRPM);
        setHoodDeg(hoodDeg);
        setTurretDeg(turretDeg);
    }

    public double calcHoodPosition(){
        //Returns hood position in degrees
        double distance = getTargetDisplacement().length();
        int keyL = getLinear(distance, dataTable);
        double hoodPosition = handleLinear(distance, dataTable[keyL][0], dataTable[keyL+1][0], dataTable[keyL][2], dataTable[keyL+1][2]);
        return hoodPosition;
    }


    public Vector2d calcBallVelocity(){
        Vector2d targetDisplacement = getTargetDisplacement();
        double targetVelocityMag = getTargetBallVelMag(targetDisplacement.length());
        Vector2d targetVelocity = new Vector2d(targetVelocityMag, 0); //Used to maintain magnitude
        targetVelocity.rotate(targetDisplacement.angle()); //Rotating it back to the correct rotation 

        Vector2d shooterVelocity = getShooterVelocity(); //Get the velocity of the shooter on the robot

        Vector2d targetBallVelocity = targetVelocity.sub(shooterVelocity); //Determine the necessary velocity of the ball being shot

        return targetBallVelocity;
    }


    public Vector2d getTargetDisplacement(){
        double targetY = targetHeight/Math.tan(camera.getTargetVerticalAngleRad());
        double targetX = targetY*Math.tan(camera.getTargetHorizontalAngleRad());
        Vector2d detectedTargetPos = new Vector2d(targetX, targetY);
        detectedTargetPos = detectedTargetPos.sub(shooterPosFromCam); //Map the detected vector onto the shooter's center
        detectedTargetPos = detectedTargetPos.rotate(getTurretAngleRad()); //This is used to rotate it back to the robot's perspective which is used to ground our measurements

        //Averaging:
        if(targetPos == null){
            //First time through
            targetPos = detectedTargetPos;
        } else {
            //Otherwise just keep averaging the position
            targetPos = targetPos.expAverage(detectedTargetPos, targetSmoothing); //Must update targetPos for next pass through the program
        }

        return targetPos;
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
           shooterVelocity = new Vector2d(0, Math.copySign(shooterVelMagnitude, robotSpeed.linearSpeed)); //Shooter is considered '90' degrees as this is the direction of the robot
        } else {
            shooterVelocity = new Vector2d(0, robotSpeed.linearSpeed);
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


    public double lawOfCosines(double _leg1, double _leg2, double angleRad){
        //Very original function
        return Math.sqrt(Math.pow(_leg1, 2) + Math.pow(_leg2, 2) - 2*_leg1*_leg2*Math.cos(angleRad));
    }

    public double getTurretAngleRad(){
        //The turret should be zeroed when facing the front of the robot and should return a positive feedback when rotated CCW
        return (turretMotor.getSelectedSensorPosition()/kQuadEncoderUnitsPerRev)*2*Math.PI;
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