package frc.robot;

import frc.robot.lib.util.ConstantsBase;

/**
 * Attribution: adapted from FRC Team 254
 */

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase {
    // singleton class
    private static Constants instance = null;

    public static Constants getInstance() {
        if (instance == null) {
            instance = new Constants();
        }
        return instance;
    }

    public static double kLoopDt = 0.01;
    public static double kDriveWatchdogTimerThreshold = 0.500;
    public static int kTalonTimeoutMs = 5; // ms
    public static int kTalonPidIdx = 0; // 0 for non-cascaded PIDs, 1 for cascaded PIDs
    
    public static double kIntakeVoltage = 0.9; //Was 0.85
    
    public static double kNominalBatteryVoltage = 12.0;


    // Hardware Port Definitions
    public static int kLeftMotorMasterTalonId =     1;
    public static int kLeftMotorSlave1TalonId =     2;
    public static int kRightMotorMasterTalonId =    3;
    public static int kRightMotorSlave1TalonId =    4;
    public static int kShooterTalonId =             31;
    public static int kShooterSlaveId =             32;
    public static int kPanelMasterId =              41;
    public static int kIntakeTalonId =              6;
    public static int kAgitatorTalonId =            7;
    

    // Gyro
    public enum GyroSelectionEnum { BNO055, NAVX, PIGEON; }
    //public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.BNO055;
    public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.NAVX;
    // public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.PIGEON;
    



    // Bumpers
    public static double kCenterToFrontBumper = 18.0; // position of front bumper with respect to robot center of
    // rotation
    public static double kCenterToExtendedIntake = 29.0; // position of intake sweetspot when extended with respect to
                                                         // robot center of rotation
    public static double kCenterToRearBumper = 17.0; // position of rear bumper with respect to robot center of rotation
    public static double kCenterToSideBumper = 18.0; // position of side bumper with respect to robot center of rotation
    public static double kCenterToCornerBumper = Math.sqrt(kCenterToRearBumper * kCenterToRearBumper + kCenterToSideBumper * kCenterToSideBumper);




    // Vision constants
    public static double kCameraFrameRate = 90.0;		// frames per second
    
    public static double kVisionMaxVel    = 20.0; // inches/sec  		
    public static double kVisionMaxAccel  = 20.0; // inches/sec^2		
    public static double kTargetWidthInches = 14.625;    
    public static double kTargetHeightInches = 6.00;
    public static double kCenterOfTargetHeightInches = 27.75;

    public static double kCargoCameraPoseX        =  0.00;	// camera location with respect to robot center of rotation, X axis is in direction of travel
    public static double kCargoCameraPoseY        =  0.00;	// camera location with respect to robot center of rotation, Y axis is positive to the left
    public static double kCargoCameraPoseZ        =  18.0;	// camera location with respect to floor, Z axis is positive with increasing elevation
    public static double kCargoCameraPoseThetaRad =   0.0;	// camera angle with respect to robot heading, in radians
    public static double kCargoCameraPitchRad     = 3.588;   // camera vertical angle with respect to level ground, in radians
    public static double kCargoCameraDeadband = 0.0;

    public static double kHatchCameraPoseX        = -11.5;	// camera location with respect to robot center of rotation, X axis is in direction of travel
    public static double kHatchCameraPoseY        =     0;	// camera location with respect to robot center of rotation, Y axis is positive to the left
    public static double kHatchCameraPoseZ        =  41.0;	// camera location with respect to floor, Z axis is positive with increasing elevation
    public static double kHatchCameraPoseThetaRad = Math.PI;	// camera angle with respect to robot heading, in radians
    public static double kHatchCameraPitchRad     = -0.3665;   // camera vertical angle with respect to level ground, in radians
    public static double kHatchCameraDeadband = 0.0;

    public static double kCargoTargetDistanceThresholdFromBumperInches = 17.0;		// inches to stop from target, measured from front bumper
    public static double kCargoTargetDistanceThresholdFromCenterInches = kCenterToFrontBumper + kCargoTargetDistanceThresholdFromBumperInches;
    
    public static double kHatchTargetDistanceThresholdFromBumperInches = 3.0;		// inches to stop from target, measured from front bumper
    public static double kHatchTargetDistanceThresholdFromCenterInches = kCenterToRearBumper + kHatchTargetDistanceThresholdFromBumperInches;
    
    public static double kVisionCompletionTolerance = 1.0; 
    public static double kVisionMaxDistanceInches = 240;		// ignore targets greater than this distance
    public static double kVisionLookaheadDist = 24.0;	// inches
    
    // Shooter Constants
    public static double kAutoAimPredictionTime =   0;	// set to 0 since we don't have a turret and need to point the entire robot








    //Intake
    public static double kzoom = 1;

    //Agitator
    public static double kAgitatorSpeed = 1;
};
