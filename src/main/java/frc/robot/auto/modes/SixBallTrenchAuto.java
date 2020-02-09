package frc.robot.auto.modes;

import frc.robot.auto.*;
import frc.robot.auto.actions.*;
import frc.robot.lib.util.*;
import frc.robot.lib.util.Path.*;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.auto.modes.*;

/**
 * Just drive in a straight line, using VelocityHeading mode
 */
public class SixBallTrenchAuto extends AutoModeBase {

    public EightBallTrenchAuto() 
    { 
    }
    

    @Override
    protected void routine() throws AutoModeEndedException 
    {
        
        int doneShooting = 3;   //three is a testing time to make sure we know that a path is about to start
        final double targetDistance;
        final int shooterStop = 0;
        final int ballCount = 3;

        double   fastSpeed = 72;
        double    medSpeed = 48;
        double   slowSpeed = 24;
        double visionSpeed = 36;

        double     accelTime = 1.0;     // time to accelerate to full speed
        double lookaheadTime = 1.0;     // time to lookahead

        PathSegment.Options   fastOptions = new PathSegment.Options(  fastSpeed,   fastSpeed/accelTime, fastSpeed/lookaheadTime, false);
        PathSegment.Options    medOptions = new PathSegment.Options(   medSpeed,    medSpeed/accelTime,  medSpeed/lookaheadTime, false);
        PathSegment.Options   slowOptions = new PathSegment.Options(  slowSpeed,   slowSpeed/accelTime, slowSpeed/lookaheadTime, false);
        PathSegment.Options visionOptions = new PathSegment.Options(visionSpeed, visionSpeed/accelTime,     visionLookaheadDist, true);


        Path backupToTrenchPath = new Path();     //lineup using limelight feed
        backupToTrenchPath.add(new Waypoint(FieldDimensions.portStartPose, fastOptions));//start line
        backupToTrenchPath.add(new Waypoint(FieldDimensions.allianceTrenchCloseEdgex, fastOptions));//turn position
        // backupToTrenchPath.add(new Waypoint(FieldDimensions.allianceStopBeforeTrenchx, visionOptions));//turn on limelights
        //may need to be reversed 

        //new path drive forward path while intaking and lineup to shoot using limelights
        Path intakeTrenchPath = new Path();
        intakeTrenchPath.add(new Waypoint(FieldDimensions.allianceTrenchCloseEdgex,medOptions));//turn on limelights end point
        intakeTrenchPath.add(new Waypoint(FieldDimensions.allianceStopBeforeTrenchx,medOptions)); //a foot away from last ball intaked
        //May need to be reversed

        // Actions--------------------------------------------------------------------------------------------------------------------------------
        //run action - shoot
        runAction(new setTurretAction());
        runAction(new SpeedUpShooterAction(targetDistance));
        runAction(new FeedBallsAction(ballCount));
        runAction(new WaitAction(doneShooting));
        runAction(new SpeedUpShooterAction(shooterStop));
        //backup path to start of trench
        runAction(new PathFollowerAction(backupToTrenchPath));
        //intake action
        runAction(new IntakeAction());
        runAction(new IntakeStopAction());
        //drive forward path while intaking
        runAction(new PathFollowerAction(intakeTrenchPath));
        //lineup turret action
        runAction(new setTurretAction());
        //run action - shoot   
        runAction(new SpeedUpShooterAction(targetDistance));
        runAction(new FeedBallsAction(ballCount));
        runAction(new WaitAction(doneShooting));
        runAction(new SpeedUpShooterAction(shooterStop)); 
                 
    }
}
