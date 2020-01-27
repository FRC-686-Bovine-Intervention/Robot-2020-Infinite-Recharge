package frc.robot.auto.modes;

import frc.robot.Constants;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;

/**
 * Interface that holds all the field measurements 
 */

public class FieldDimensions 
{
    public static boolean rightSide = false;

    public enum TargetPositionEnum
    {
        CARGO_FRONT, CARGO_SIDE1, CARGO_SIDE2, CARGO_SIDE3, ROCKET_NEAR, ROCKET_FAR;
    }

	// dimensions of field components
	public static double kFieldLengthX = 648;       // 54'
	public static double kFieldLengthY = 324;       // 27'
    
    // Habitat
    public static double kHabWidthY = 128.0;        // not counting ramp
    public static double kHab3DepthX = 48.0;

    // // Robot starting poses
	// public static Pose centerLeftStartPose  = new Pose(kHab3DepthX + Constants.kCenterToFrontBumper, 0,                                             Math.PI);
	// public static Pose centerRightStartPose = new Pose(kHab3DepthX + Constants.kCenterToFrontBumper, 0,                                             Math.PI);	
	// public static Pose leftStartPose        = new Pose(kHab3DepthX + Constants.kCenterToFrontBumper, +kHabWidthY/2 - Constants.kCenterToSideBumper, 0);  // side of robot aligned with edge of HAB
	// public static Pose rightStartPose       = new Pose(kHab3DepthX + Constants.kCenterToFrontBumper, -kHabWidthY/2 + Constants.kCenterToSideBumper, 0);  // side of robot aligned with edge of HAB
	// public static Pose leftHab2StartPose    = new Pose(Constants.kCenterToFrontBumper, +kHabWidthY/2 - Constants.kCenterToSideBumper, 0);  // side of robot aligned with edge of HAB
	// public static Pose rightHab2StartPose   = new Pose(Constants.kCenterToFrontBumper, -kHabWidthY/2 + Constants.kCenterToSideBumper, 0);  // side of robot aligned with edge of HAB

    // Rocket
    private static Vector2d kRocketCenter = new Vector2d(229.1, 153.3);
    private static double   kRocketCenterToHatch = 17.0;                         // distance from rocket center to center of hatches on diagonal
    private static double   kRocketAngleRad = 30.0*Vector2d.degreesToRadians ;   // angle from center to hatch, relative to back
    private static double   kRocketVisionDist1 = 60.0;                            // distance from which to turn on cameras
    private static double   kRocketVisionDist2 = 30.0;                            // distance from which to turn on cameras
    private static double   kRocketTurnDist   = 30.0;                            // distance from which to turn towards rocket
    private static double   kRocketBackupTurnDist   = 30.0;                      
    
    private static double   kNearRocketAngleRad = Math.PI - kRocketAngleRad;
    private static double   kFarRocketAngleRad =      0.0 - kRocketAngleRad;
    private static double   kNearRocketBackupTurnAngleRad = kNearRocketAngleRad;
    private static double   kFarRocketBackupTurnAngleRad = kFarRocketAngleRad;

    // Near Rocket
    private static Vector2d kNearRocketHatchPosition  = kRocketCenter.add( Vector2d.magnitudeAngle(kRocketCenterToHatch, kNearRocketAngleRad) );
    private static Vector2d kNearRocketVisionPosition2 = kNearRocketHatchPosition.add(  Vector2d.magnitudeAngle(kRocketVisionDist2, kNearRocketAngleRad) );
    private static Vector2d kNearRocketVisionPosition1 = kNearRocketHatchPosition.add(  Vector2d.magnitudeAngle(kRocketVisionDist1, kNearRocketAngleRad) );
    private static Vector2d kNearRocketTurnPosition   = kNearRocketVisionPosition1.add( Vector2d.magnitudeAngle(kRocketTurnDist,   kNearRocketAngleRad) );
    private static Vector2d kNearRocketBackupTurnPosition   = kNearRocketTurnPosition.add( Vector2d.magnitudeAngle(kRocketBackupTurnDist,   kNearRocketBackupTurnAngleRad) );
 
    private static double   kNearRocketBackupDist1   = 6.0;                  // backup1: just back up a little away from hatch
    private static double   kNearRocketBackupAngle1  = kNearRocketAngleRad; 
    private static double   kNearRocketBackupDist2   = 72.0;                 // backup2: fast backup 6 ft towards center of field
    private static double   kNearRocketBackupAngle2  = -Math.PI/2;                           
    private static double   kNearRocketBackupDist3   = 1.0;                  // backup3: just to keep robot pointed in right direction
    private static double   kNearRocketBackupAngle3  = 0.0;                           

    private static Vector2d kNearRocketBackupPosition1 = kNearRocketHatchPosition.add(   Vector2d.magnitudeAngle(kNearRocketBackupDist1, kNearRocketBackupAngle1));
    private static Vector2d kNearRocketBackupPosition2 = kNearRocketBackupPosition1.add( Vector2d.magnitudeAngle(kNearRocketBackupDist2, kNearRocketBackupAngle2));
    private static Vector2d kNearRocketBackupPosition3 = kNearRocketBackupPosition2.add( Vector2d.magnitudeAngle(kNearRocketBackupDist3, kNearRocketBackupAngle3));

    // Far Rocket
    private static Vector2d kFarRocketHatchPosition  = kRocketCenter.add( Vector2d.magnitudeAngle(kRocketCenterToHatch, kFarRocketAngleRad) );
    private static Vector2d kFarRocketVisionPosition2 = kFarRocketHatchPosition.add(  Vector2d.magnitudeAngle(kRocketVisionDist2, kFarRocketAngleRad) );
    private static Vector2d kFarRocketVisionPosition1 = kFarRocketHatchPosition.add(  Vector2d.magnitudeAngle(kRocketVisionDist1, kFarRocketAngleRad) );
    private static Vector2d kFarRocketTurnPosition   = kFarRocketVisionPosition1.add( Vector2d.magnitudeAngle(kRocketTurnDist,   kFarRocketAngleRad) );
    private static Vector2d kFarRocketBackupTurnPosition   = kFarRocketTurnPosition.add( Vector2d.magnitudeAngle(kRocketBackupTurnDist,   kFarRocketBackupTurnAngleRad) );
 
    private static double   kFarRocketBackupDist1   = 6.0;                  // backup1: just back up a little away from hatch
    private static double   kFarRocketBackupAngle1  = kFarRocketAngleRad; 
    private static double   kFarRocketBackupDist2   = 72.0;                 // backup2: fast backup 6 ft towards center of field
    private static double   kFarRocketBackupAngle2  = -Math.PI/2;                           
    private static double   kFarRocketBackupDist3   = 1.0;                  // backup3: just to keep robot pointed in right direction
    private static double   kFarRocketBackupAngle3  = 0.0;                           

    private static Vector2d kFarRocketBackupPosition1 = kFarRocketHatchPosition.add(   Vector2d.magnitudeAngle(kFarRocketBackupDist1, kFarRocketBackupAngle1));
    private static Vector2d kFarRocketBackupPosition2 = kFarRocketBackupPosition1.add( Vector2d.magnitudeAngle(kFarRocketBackupDist2, kFarRocketBackupAngle2));
    private static Vector2d kFarRocketBackupPosition3 = kFarRocketBackupPosition2.add( Vector2d.magnitudeAngle(kFarRocketBackupDist3, kFarRocketBackupAngle3));





    // Cargo Ship
    private static double   kCargoVisionDist1 = 60.0;                      // distance from which to turn on cameras (desired target should be mostly centered)
    private static double   kCargoVisionDist2 = 30.0;                      // midway point from hatch to Vision1
    private static double   kCargoTurnDist    = 30.0;                      // distance from which to turn towards cargo bay
    private static double   kCargoBackupTurnDist   = 30.0;                      // distance from which to turn towards cargo bay

    // Cargo Ship Front Bay
    private static Vector2d kCargoFrontHatchPosition = new Vector2d(220.3, 10.9);
    private static double   kCargoFrontAngleRad = -Math.PI;
    private static double   kCargoFrontBackupTurnAngleRad = -Math.PI/2;

    private static Vector2d kCargoFrontVisionPosition2  = kCargoFrontHatchPosition.add( Vector2d.magnitudeAngle(kCargoVisionDist2, kCargoFrontAngleRad) );
    private static Vector2d kCargoFrontVisionPosition1  = kCargoFrontHatchPosition.add( Vector2d.magnitudeAngle(kCargoVisionDist1, kCargoFrontAngleRad) );
    private static Vector2d kCargoFrontTurnPosition    = kCargoFrontVisionPosition1.add( Vector2d.magnitudeAngle(kCargoTurnDist,  kCargoFrontAngleRad) );
    private static Vector2d kCargoFrontBackupTurnPosition   = kCargoFrontTurnPosition.add( Vector2d.magnitudeAngle(kCargoBackupTurnDist,   kCargoFrontBackupTurnAngleRad) );
    
    private static double   kCargoFrontBackupDist1   = 48.0;                  // backup1: just back up a little away from hatch
    private static double   kCargoFrontBackupAngle1  = kCargoFrontAngleRad; 
    private static double   kCargoFrontBackupDist2   = 72.0;                 // backup2: fast backup 6 ft towards center of field
    private static double   kCargoFrontBackupAngle2  = +Math.PI/2;                           
    private static double   kCargoFrontBackupDist3   = 1.0;                  // backup3: just to keep robot pointed in right direction
    private static double   kCargoFrontBackupAngle3  = 0.0;                           
    
    private static Vector2d kCargoFrontBackupPosition1 = kCargoFrontHatchPosition.add(   Vector2d.magnitudeAngle(kCargoFrontBackupDist1, kCargoFrontBackupAngle1));
    private static Vector2d kCargoFrontBackupPosition2 = kCargoFrontBackupPosition1.add( Vector2d.magnitudeAngle(kCargoFrontBackupDist2, kCargoFrontBackupAngle2));
    private static Vector2d kCargoFrontBackupPosition3 = kCargoFrontBackupPosition2.add( Vector2d.magnitudeAngle(kCargoFrontBackupDist3, kCargoFrontBackupAngle3));
    
    
    // Cargo Ship Side Bays
    private static Vector2d kCargoSide1HatchPosition = new Vector2d(260.8, 27.9);
    private static Vector2d kCargoSide2HatchPosition = new Vector2d(282.6, 27.9);
    private static Vector2d kCargoSide3HatchPosition = new Vector2d(304.3, 27.9);
    private static double   kCargoSideAngleRad = +Math.PI/2;
    private static double   kCargoSideBackupTurnAngleRad = 0;
    
    private static Vector2d kCargoSide1VisionPosition2 = kCargoSide1HatchPosition.add(  Vector2d.magnitudeAngle(kCargoVisionDist2, kCargoSideAngleRad) );
    private static Vector2d kCargoSide2VisionPosition2 = kCargoSide2HatchPosition.add(  Vector2d.magnitudeAngle(kCargoVisionDist2, kCargoSideAngleRad) );
    private static Vector2d kCargoSide3VisionPosition2 = kCargoSide3HatchPosition.add(  Vector2d.magnitudeAngle(kCargoVisionDist2, kCargoSideAngleRad) );
    
    private static Vector2d kCargoSide1VisionPosition1 = kCargoSide1HatchPosition.add(  Vector2d.magnitudeAngle(kCargoVisionDist1, kCargoSideAngleRad) );
    private static Vector2d kCargoSide2VisionPosition1 = kCargoSide2HatchPosition.add(  Vector2d.magnitudeAngle(kCargoVisionDist1, kCargoSideAngleRad) );
    private static Vector2d kCargoSide3VisionPosition1 = kCargoSide3HatchPosition.add(  Vector2d.magnitudeAngle(kCargoVisionDist1, kCargoSideAngleRad) );
    
    private static Vector2d kCargoSide1TurnPosition   = kCargoSide1VisionPosition1.add( Vector2d.magnitudeAngle(kCargoTurnDist,   kCargoSideAngleRad));
    private static Vector2d kCargoSide2TurnPosition   = kCargoSide2VisionPosition1.add( Vector2d.magnitudeAngle(kCargoTurnDist,   kCargoSideAngleRad));
    private static Vector2d kCargoSide3TurnPosition   = kCargoSide3VisionPosition1.add( Vector2d.magnitudeAngle(kCargoTurnDist,   kCargoSideAngleRad));
    
    private static Vector2d kCargoSide1BackupTurnPosition   = kCargoSide1TurnPosition.add( Vector2d.magnitudeAngle(kCargoBackupTurnDist,   kCargoSideBackupTurnAngleRad) );
    private static Vector2d kCargoSide2BackupTurnPosition   = kCargoSide2TurnPosition.add( Vector2d.magnitudeAngle(kCargoBackupTurnDist,   kCargoSideBackupTurnAngleRad) );
    private static Vector2d kCargoSide3BackupTurnPosition   = kCargoSide3TurnPosition.add( Vector2d.magnitudeAngle(kCargoBackupTurnDist,   kCargoSideBackupTurnAngleRad) );

    private static double   kCargoSideBackupDist1   = 6.0;                  // backup1: just back up a little away from hatch
    private static double   kCargoSideBackupAngle1  = kCargoSideAngleRad; 
    private static double   kCargoSideBackupDist2   = 36.0;                 // backup2: fast backup 3 ft towards center of field
    private static double   kCargoSideBackupAngle2  = kCargoSideAngleRad;                           
    private static double   kCargoSideBackupDist3   = 36.0;                 // backup3: point towards human station
    private static double   kCargoSideBackupAngle3  = 0.0;                           
    
    private static Vector2d kCargoSide1BackupPosition1 = kCargoSide1HatchPosition.add(  Vector2d.magnitudeAngle(kCargoSideBackupDist1,   kCargoSideBackupAngle1) );
    private static Vector2d kCargoSide2BackupPosition1 = kCargoSide2HatchPosition.add(  Vector2d.magnitudeAngle(kCargoSideBackupDist1,   kCargoSideBackupAngle1) );
    private static Vector2d kCargoSide3BackupPosition1 = kCargoSide3HatchPosition.add(  Vector2d.magnitudeAngle(kCargoSideBackupDist1,   kCargoSideBackupAngle1) );
    
    private static Vector2d kCargoSide1BackupPosition2 = kCargoSide1BackupPosition1.add(Vector2d.magnitudeAngle(kCargoSideBackupDist2,   kCargoSideBackupAngle2) );
    private static Vector2d kCargoSide2BackupPosition2 = kCargoSide2BackupPosition1.add(Vector2d.magnitudeAngle(kCargoSideBackupDist2,   kCargoSideBackupAngle2) );
    private static Vector2d kCargoSide3BackupPosition2 = kCargoSide3BackupPosition1.add(Vector2d.magnitudeAngle(kCargoSideBackupDist2,   kCargoSideBackupAngle2) );

    private static Vector2d kCargoSide1BackupPosition3 = kCargoSide1BackupPosition2.add(Vector2d.magnitudeAngle(kCargoSideBackupDist3,   kCargoSideBackupAngle3) );
    private static Vector2d kCargoSide2BackupPosition3 = kCargoSide2BackupPosition2.add(Vector2d.magnitudeAngle(kCargoSideBackupDist3,   kCargoSideBackupAngle3) );
    private static Vector2d kCargoSide3BackupPosition3 = kCargoSide3BackupPosition2.add(Vector2d.magnitudeAngle(kCargoSideBackupDist3,   kCargoSideBackupAngle3) );

    


    // Human Station
    private static Vector2d kHumanStationHatchPosition = new Vector2d(-4.0, 136.3); //0.0
    private static double   kHumanStationAngleRad = 0.0;

    private static double   kHumanStationVisionDist1 = 60.0;                      // distance from which to turn turn on cameras (desired target should be mostly centered)
    private static double   kHumanStationVisionDist2 = 30.0;                      // distance from which to turn turn on cameras (desired target should be mostly centered)
    private static double   kHumanStationTurnDist   = 48.0;                      // distance from which to turn towards human station
    
    private static Vector2d kHumanStationFarRocketMidPosition  = new Vector2d(200, 110);     // position to pass through when going FarRocket<-->HumanStation
    private static Vector2d kHumanStationSideCargoMidPosition  = new Vector2d(200, 110);     // position to pass through when going CargoSide<-->HumanStation
    private static Vector2d kHumanStationFrontCargoMidPosition = new Vector2d(130, 100);     // position to pass through when going HumanStation-->CargoFront

    private static Vector2d kHumanStationVisionPosition2 = kHumanStationHatchPosition.add(  Vector2d.magnitudeAngle(kHumanStationVisionDist2, kHumanStationAngleRad) );
    private static Vector2d kHumanStationVisionPosition1 = kHumanStationHatchPosition.add(  Vector2d.magnitudeAngle(kHumanStationVisionDist1, kHumanStationAngleRad) );
    private static Vector2d kHumanStationTurnPosition =   kHumanStationVisionPosition1.add( Vector2d.magnitudeAngle(kHumanStationTurnDist,   kHumanStationAngleRad-20*Math.PI/180) );



    
    // gets
    
    // // rightSide is set to true if the chosen start pose is on the right
	// public static Pose getHab1CenterLeftStartPose()     { rightSide = false;   return centerLeftStartPose; }	
	// public static Pose getHab1CenterRightStartPose()    { rightSide =  true;   return centerRightStartPose; }	
	// public static Pose getHab1LeftStartPose()           { rightSide = false;   return leftStartPose; }  
    // public static Pose getHab1RightStartPose()          { rightSide =  true;   return rightStartPose; } 
	// public static Pose getHab2LeftStartPose()           { rightSide = false;   return leftHab2StartPose; }  
    // public static Pose getHab2RightStartPose()          { rightSide =  true;   return rightHab2StartPose; } 
    
    public static Vector2d getCargoFrontSpacing()
    {
        Vector2d rv = kCargoFrontHatchPosition.sub(kCargoFrontHatchPosition.conj());    // (0, +21.75) if target1 on right, target2 on left
        if (!rightSide) {
            rv = rv.conj(); }                                                           // (0, -21.75) if target1 on left, target2 on right
        return rv;
    }
    
    public static Vector2d getHumanStationTurnPosition()   { return (!rightSide ? kHumanStationTurnPosition   : kHumanStationTurnPosition.conj()); }
    public static Vector2d getHumanStationVisionPosition1() { return (!rightSide ? kHumanStationVisionPosition1 : kHumanStationVisionPosition1.conj()); }
    public static Vector2d getHumanStationVisionPosition2() { return (!rightSide ? kHumanStationVisionPosition1 : kHumanStationVisionPosition2.conj()); }
    public static Vector2d getHumanStationHatchPosition()  { return (!rightSide ? kHumanStationHatchPosition  : kHumanStationHatchPosition.conj()); }

    public static Vector2d getHumanStationFarRocketMidPosition()  { return (!rightSide ? kHumanStationFarRocketMidPosition  : kHumanStationFarRocketMidPosition.conj()); }
    public static Vector2d getHumanStationSideCargoMidPosition()  { return (!rightSide ? kHumanStationSideCargoMidPosition  : kHumanStationSideCargoMidPosition.conj()); }
    public static Vector2d getHumanStationFrontCargoMidPosition() { return (!rightSide ? kHumanStationFrontCargoMidPosition : kHumanStationFrontCargoMidPosition.conj()); }


    public static Vector2d getTargetBackupTurnPosition(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
       switch (_target)
        {
            case CARGO_FRONT:   rv = kCargoFrontBackupTurnPosition; break;
            case CARGO_SIDE1:   rv = kCargoSide1BackupTurnPosition; break;
            case CARGO_SIDE2:   rv = kCargoSide2BackupTurnPosition; break;
            case CARGO_SIDE3:   rv = kCargoSide3BackupTurnPosition; break;
            case ROCKET_NEAR:   rv = kNearRocketBackupTurnPosition; break;
            case ROCKET_FAR:    rv =  kFarRocketBackupTurnPosition; break;
        }
        if (rightSide)
        {
            rv = rv.conj();
        }
        return rv;
    }

    public static Vector2d getTargetTurnPosition(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
       switch (_target)
        {
            case CARGO_FRONT:   rv = kCargoFrontTurnPosition; break;
            case CARGO_SIDE1:   rv = kCargoSide1TurnPosition; break;
            case CARGO_SIDE2:   rv = kCargoSide2TurnPosition; break;
            case CARGO_SIDE3:   rv = kCargoSide3TurnPosition; break;
            case ROCKET_NEAR:   rv = kNearRocketTurnPosition; break;
            case ROCKET_FAR:    rv =  kFarRocketTurnPosition; break;
        }
        if (rightSide)
        {
            rv = rv.conj();
        }
        return rv;
    }

    public static Vector2d getTargetVisionPosition1(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
        switch (_target)
        {
            case CARGO_FRONT:   rv = kCargoFrontVisionPosition1; break;
            case CARGO_SIDE1:   rv = kCargoSide1VisionPosition1; break;
            case CARGO_SIDE2:   rv = kCargoSide2VisionPosition1; break;
            case CARGO_SIDE3:   rv = kCargoSide3VisionPosition1; break;
            case ROCKET_NEAR:   rv = kNearRocketVisionPosition1; break;
            case ROCKET_FAR:    rv =  kFarRocketVisionPosition1; break;
        }
        if (rightSide)
        {
            rv = rv.conj();
        }
        return rv;
    }

    public static Vector2d getTargetVisionPosition2(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
        switch (_target)
        {
            case CARGO_FRONT:   rv = kCargoFrontVisionPosition2; break;
            case CARGO_SIDE1:   rv = kCargoSide1VisionPosition2; break;
            case CARGO_SIDE2:   rv = kCargoSide2VisionPosition2; break;
            case CARGO_SIDE3:   rv = kCargoSide3VisionPosition2; break;
            case ROCKET_NEAR:   rv = kNearRocketVisionPosition2; break;
            case ROCKET_FAR:    rv =  kFarRocketVisionPosition2; break;
        }
        if (rightSide)
        {
            rv = rv.conj();
        }
        return rv;
    }

    public static Vector2d getTargetHatchPosition(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();

        switch (_target)
        {
            case CARGO_FRONT:   rv = kCargoFrontHatchPosition; break;
            case CARGO_SIDE1:   rv = kCargoSide1HatchPosition; break;
            case CARGO_SIDE2:   rv = kCargoSide2HatchPosition; break;
            case CARGO_SIDE3:   rv = kCargoSide3HatchPosition; break;
            case ROCKET_NEAR:   rv = kNearRocketHatchPosition; break;
            case ROCKET_FAR:    rv =  kFarRocketHatchPosition; break;
        }
        if (rightSide)
        {
            rv = rv.conj();
        }
        return rv;
    }

    public static Vector2d getTargetBackupPosition1(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
        switch (_target)
        {
            case CARGO_FRONT:   rv = kCargoFrontBackupPosition1; break;
            case CARGO_SIDE1:   rv = kCargoSide1BackupPosition1; break;
            case CARGO_SIDE2:   rv = kCargoSide2BackupPosition1; break;
            case CARGO_SIDE3:   rv = kCargoSide3BackupPosition1; break;
            case ROCKET_NEAR:   rv = kNearRocketBackupPosition1; break;
            case ROCKET_FAR:    rv =  kFarRocketBackupPosition1; break;
        }
        if (rightSide)
        {
            rv = rv.conj();
        }
        return rv;
    }

    public static Vector2d getTargetBackupPosition2(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
        switch (_target)
        {
            case CARGO_FRONT:   rv = kCargoFrontBackupPosition2; break;
            case CARGO_SIDE1:   rv = kCargoSide1BackupPosition2; break;
            case CARGO_SIDE2:   rv = kCargoSide2BackupPosition2; break;
            case CARGO_SIDE3:   rv = kCargoSide3BackupPosition2; break;
            case ROCKET_NEAR:   rv = kNearRocketBackupPosition2; break;
            case ROCKET_FAR:    rv =  kFarRocketBackupPosition2; break;
        }
        if (rightSide)
        {
            rv = rv.conj();
        }
        return rv;
    }

    public static Vector2d getTargetBackupPosition3(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
        switch (_target)
        {
            case CARGO_FRONT:   rv = kCargoFrontBackupPosition3; break;
            case CARGO_SIDE1:   rv = kCargoSide1BackupPosition3; break;
            case CARGO_SIDE2:   rv = kCargoSide2BackupPosition3; break;
            case CARGO_SIDE3:   rv = kCargoSide3BackupPosition3; break;
            case ROCKET_NEAR:   rv = kNearRocketBackupPosition3; break;
            case ROCKET_FAR:    rv =  kFarRocketBackupPosition3; break;
        }
        if (rightSide)
        {
            rv = rv.conj();
        }
        return rv;
    }
    
    public static Vector2d getRobotPositionAtTarget(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        switch (_target)
        {
            // case CARGO_FRONT:   rv = kCargoFrontHatchPosition.add( Vector2d.magnitudeAngle(Constants.kCenterToRearBumper, kCargoFrontAngleRad)); break;
            // case CARGO_SIDE1:   rv = kCargoSide1HatchPosition.add( Vector2d.magnitudeAngle(Constants.kCenterToRearBumper, kCargoSideAngleRad)); break;
            // case CARGO_SIDE2:   rv = kCargoSide2HatchPosition.add( Vector2d.magnitudeAngle(Constants.kCenterToRearBumper, kCargoSideAngleRad)); break;
            // case CARGO_SIDE3:   rv = kCargoSide3HatchPosition.add( Vector2d.magnitudeAngle(Constants.kCenterToRearBumper, kCargoSideAngleRad)); break;
            // case ROCKET_NEAR:   rv = kNearRocketHatchPosition.add( Vector2d.magnitudeAngle(Constants.kCenterToRearBumper, kNearRocketAngleRad)); break;
            // case ROCKET_FAR:    rv =  kFarRocketHatchPosition.add( Vector2d.magnitudeAngle(Constants.kCenterToRearBumper,  kFarRocketAngleRad)); break;
        }
        if (rightSide)
        {
            rv = rv.conj();
        }
        return rv;
    }

    public static Vector2d getRobotPositionAtHumanStation()
    {
        Vector2d rv = new Vector2d();
        //rv =  kHumanStationHatchPosition.add(Vector2d.magnitudeAngle(Constants.kHatchTargetDistanceThresholdFromCenterInches, kHumanStationAngleRad));
        if (rightSide)
        {
            rv = rv.conj();
        }
        return rv;
    }

	private static final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			// put("LeftStartPoseX", getHab1LeftStartPose().getX());
            // put("LeftStartPoseY", getHab1LeftStartPose().getY());
            
        
			// put("kCargoSide1TurnPositionX", kCargoSide1TurnPosition.getX());
			// put("kCargoSide1TurnPositionY", kCargoSide1TurnPosition.getY());

            // put("kCargoSide1VisionPosition", kCargoSide1VisionPosition.getX());
            // put("kCargoSide1VisionPosition", kCargoSide1VisionPosition.getY());
            
			// put("kCargoSide1HatchPositionX", kCargoSide1HatchPosition.getX());
			// put("kCargoSide1HatchPositionY", kCargoSide1HatchPosition.getY());

            // put("kCargoSide1BackupPosition", kCargoSide1BackupPosition.getX());
			// put("kCargoSide1BackupPosition", kCargoSide1BackupPosition.getY());
        }
    };
    
    public static DataLogger getLogger() { return logger; }    
}