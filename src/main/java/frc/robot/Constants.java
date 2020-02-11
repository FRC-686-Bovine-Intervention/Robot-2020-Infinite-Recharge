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
        
    public static double kNominalBatteryVoltage = 12.0;


    // Hardware Port Definitions
    public static int kPCMID =                      01;
    public static int kRightMotorMasterTalonId =    11;
    public static int kRightMotorSlave1TalonId =    12;
    public static int kLeftMotorMasterTalonId =     13;
    public static int kLeftMotorSlave1TalonId =     14;
    public static int kIntakeTalonId =              21;
    public static int kConveyorbeltMasterID =       22;
    public static int kConveyorbeltSlaveID =        23;
    public static int kShooterTalonId =             31;
    public static int kShooterSlaveId =             32;
    public static int kShooterTurretID =            33;
    public static int kShooterHoodID =              34;
    public static int kPanelMasterId =              41;
    public static int kSliderMasterId =             42;

    public static int kMainSolenoidChannel =        0;
    public static int kSecondarySolenoidChannel =   1;
    public static int kPTOSolenoidChannel =         2;
    public static int kLiftLockSolenoidChannel =    3;
    
    public static int kLiftSensorChannel =          1010;
    public static int kPanelUltrasonicChannel =     10000;
    public static int kPanelLeftTouchChannel =      100101;
    public static int kPanelRightTouchChannel =     10101;
    
    public static double kIntakeVoltage = 0.5;
    public static double kConveyorRPM = 30.0;














    // Gyro
    public enum GyroSelectionEnum { BNO055, NAVX, PIGEON; }
    //public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.BNO055;
    public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.NAVX;
    // public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.PIGEON;



    // Vision constants
    public static double kCameraFrameRate = 90.0;		// frames per second
    
    public static double kVisionMaxVel    = 20.0; // inches/sec  		
    public static double kVisionMaxAccel  = 20.0; // inches/sec^2		
    public static double kTargetWidthInches = 14.625;    
    public static double kTargetHeightInches = 6.00;
    public static double kCenterOfTargetHeightInches = 27.75;
    
    public static double kVisionCompletionTolerance = 1.0; 
    public static double kVisionMaxDistanceInches = 240;		// ignore targets greater than this distance
    public static double kVisionLookaheadDist = 24.0;	// inches
    
    // Shooter Constants
    public static double kAutoAimPredictionTime =   0;	// set to 0 since we don't have a turret and need to point the entire robot





}
