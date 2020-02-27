package frc.robot.auto.modes;

import frc.robot.SmartDashboardInteractions;
import frc.robot.auto.*;
import frc.robot.auto.actions.*;
import frc.robot.lib.util.*;
import frc.robot.subsystems.*;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.sensors.Pigeon;


/**
 * Just drive in a straight line, using VelocityHeading mode
 */
public class SixBallAuto extends AutoModeBase {

    public SixBallAuto() 
    { 
    }
        Shooter shooter = Shooter.getInstance();
        Pigeon pigeon = (Pigeon)Pigeon.getInstance();
        private SmartDashboardInteractions smartDashboard = SmartDashboardInteractions.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException 
    {
        double   fastSpeed = 72;
        double    medSpeed = 48;
        double   slowSpeed = 24;
        double visionSpeed = 36;

        double     accelTime = 1.0;     // time to accelerate to full speed
        double lookaheadTime = 1.0;     // time to lookahead

        double visionLookaheadDist = 24;

        PathSegment.Options   fastOptions = new PathSegment.Options(  fastSpeed,   fastSpeed/accelTime, fastSpeed/lookaheadTime, false);
        PathSegment.Options    medOptions = new PathSegment.Options(   medSpeed,    medSpeed/accelTime,  medSpeed/lookaheadTime, false);
        PathSegment.Options   slowOptions = new PathSegment.Options(  slowSpeed,   slowSpeed/accelTime, slowSpeed/lookaheadTime, false);
        PathSegment.Options visionOptions = new PathSegment.Options(visionSpeed, visionSpeed/accelTime,     visionLookaheadDist, true);

        smartDashboard = SmartDashboardInteractions.getInstance();
        Pose startPose = smartDashboard.getSelectedStartPose();
        Vector2d startPosition = startPose.getPosition();
        
        double startDelaySec = smartDashboard.getStartDelay();

        Path backupToTrenchPath = new Path();     //lineup using limelight feed
        backupToTrenchPath.add(new Waypoint(startPosition, fastOptions));//start line
        backupToTrenchPath.add(new Waypoint(FieldDimensions.allianceTrenchClosePos,fastOptions));//turn position
        //backupToTrenchPath.add(new Waypoint());//turn on limelights
        //may need to be reversed 

        //new path drive forward path while intaking and lineup to shoot using limelights
        Path intakeTrenchPath = new Path();
        intakeTrenchPath.add(new Waypoint(FieldDimensions.allianceTrenchClosePos,fastOptions));//turn on limelights end point
        intakeTrenchPath.add(new Waypoint(FieldDimensions.allianceTrenchFarPos,fastOptions)); //a foot away from last ball intaked
        //May need to be reversed
        Vector2d backUpTargetPosStart = FieldDimensions.portPos.sub(startPose.getPosition());
        // Actions--------------------------------------------------------------------------------------------------------------------------------
        //run action - shoot
        
        runAction(new WaitAction(startDelaySec));
        AimShooterAction aimShooterAction1 = new AimShooterAction();
        runAction(aimShooterAction1);
        Vector2d targetPos = aimShooterAction1.getSensedTargetPos();
        if(targetPos != null){
            runAction(new SpeedUpShooterAction(targetPos.length()));
        } else {
            runAction(new SpeedUpShooterAction(backUpTargetPosStart.length()));
        }
        runAction(new FeedBallsAction(3));
        runAction(new StopShooterAction());
        //backup path to start of trench
        runAction(new PathFollowerAction(backupToTrenchPath));
        //intake action
        runAction(new IntakeAction());
        runAction(new IntakeStopAction());
        //drive forward path while intaking
        runAction(new PathFollowerAction(intakeTrenchPath));
        //lineup turret action
        runAction(new setTurretAction());
        //run action - shoot    runAction(new SpeedUpShooterAction(targetRPM));
        //Finding Target
        Vector2d backUpTargetPosTrench = FieldDimensions.portPos.sub(FieldDimensions.allianceTrenchFarPos);
        AimShooterAction aimShooterAction2 = new AimShooterAction(backUpTargetPosTrench.angle()-pigeon.getHeadingDeg());
        runAction(aimShooterAction2);
        targetPos = null; //Resetting so that the last taken measurement won't affect the following code
        targetPos = aimShooterAction2.getSensedTargetPos();

        //Determining if the speed should be changed and shooting:
        if(targetPos != null){
            runAction(new SpeedUpShooterAction(targetPos.length()));
        } else {
            runAction(new SpeedUpShooterAction(backUpTargetPosTrench.length()));
        }
        runAction(new FeedBallsAction(3));
        runAction(new StopShooterAction());
                 
    }
}
