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

    private FieldDimensions(){}
    public enum TargetPositionEnum
    {

    }

    //Red is to the right (postive x)

	// dimensions of field components
	public static double kFieldLengthX = 648;       // 54'
	public static double kFieldLengthY = 324;       // 27'
    

    //Start Positions
    //Port is also one of these but is located elsewhere
    public static double middleStartPosy = 180.0;
    public static double playerStationy = 119.0;

    //Field-based positions
    public static double startLineDist = 149.188;
    public static double portY = 247.00;
    public static double opponentTrenchBallx = 278.456;
    public static double opponentTrenchBally = 64.705;
    public static double allianceTrenchCloseEdgex = 234.817;
    public static double allianceTrenchFarEdgex = 448.817;
    public static double allianceStopBeforeTrenchx = 342.834;
    public static double allianceTrenchMiddley = 313.905;

    //Balls
    public static double centerBlueSideBallx = 268.728;
    public static double centerBlueSideBally = 165.930;
    public static double centerBlueSideMiddleBallx = 262.39;
    public static double centerBlueSideMiddleBally = 181.227;
    public static double centerBlueSidePostBallx = 256.053;
    public static double centerBlueSidePostBally = 196.524;
    public static double centerRedSidePostBallx = 263.174;
    public static double centerRedSidePostBally = 220.930;
    public static double centerRedSideFarBallx = 278.473;
    public static double centerRedSideFarBally = 227.267;



    public static Vector2d portStartPos = new Vector2d(startLineDist, portY);
    public static Vector2d opponentTrenchBallPos = new Vector2d(opponentTrenchBallx, opponentTrenchBally);
    public static Vector2d allianceTrenchClosePos = new Vector2d(allianceTrenchCloseEdgex, allianceTrenchMiddley);
    public static Vector2d allianceTrenchFarPos = new Vector2d(allianceStopBeforeTrenchx, allianceTrenchMiddley);

    //Balls
    public static Vector2d centerBlueSideBallPos = new Vector2d(centerBlueSideBallx, centerBlueSideBally);
    public static Vector2d centerBlueSideMiddleBallPos = new Vector2d(centerBlueSideMiddleBallx, centerBlueSideMiddleBally);
    public static Vector2d centerBlueSidePosBallPos = new Vector2d(centerBlueSidePostBallx, centerBlueSidePostBally);
    public static Vector2d centerRedSidePostBallPos = new Vector2d(centerRedSidePostBallx, centerRedSidePostBally);
    public static Vector2d centerRedSideFarBallPos = new Vector2d(centerRedSideFarBallx, centerRedSideFarBally);

    //start positions part 2
    public static Pose portStartPose = new Pose(startLineDist, portY, 0.0);
    public static Pose middleStartPose = new Pose(startLineDist, middleStartPosy, 0.0);
    public static Pose playerStationStartPose = new Pose(startLineDist, playerStationy, 0.0);




	private static final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        }
    };
    
    public static DataLogger getLogger() { return logger; }    
}