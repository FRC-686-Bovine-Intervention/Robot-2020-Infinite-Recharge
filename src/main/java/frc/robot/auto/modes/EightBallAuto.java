package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.SmartDashboardInteractions;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.AimShooterAction;
import frc.robot.auto.actions.FeedBallsAction;
import frc.robot.auto.actions.IntakeAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.SpeedUpShooterAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.sensors.Pigeon;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.subsystems.Shooter;

public class EightBallAuto extends AutoModeBase {

    public EightBallAuto() {
    }

    private Shooter shooter = Shooter.getInstance();
    private Pigeon pigeon = (Pigeon)Pigeon.getInstance();
    private SmartDashboardInteractions smartDashboard = SmartDashboardInteractions.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException {

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



        //Paths
        Path startToMid = new Path();
        startToMid.add(new Waypoint(startPosition, fastOptions));
        startToMid.add(new Waypoint(FieldDimensions.midApproachPos, medOptions));
        
        Path ballCollectionPath = new Path();
        ballCollectionPath.add(new Waypoint(FieldDimensions.midApproachPos, medOptions));
        ballCollectionPath.add(new Waypoint(FieldDimensions.centerBlueSideBallPos, medOptions));
        ballCollectionPath.add(new Waypoint(FieldDimensions.centerBlueSideMiddleBallPos, medOptions));
        ballCollectionPath.add(new Waypoint(FieldDimensions.centerBlueSidePostBallPos, medOptions));
        ballCollectionPath.add(new Waypoint(FieldDimensions.centerRedSidePostBallPos, medOptions));
        ballCollectionPath.add(new Waypoint(FieldDimensions.centerRedSideFarBallPos, medOptions));

        Path moveToShootPath = new Path();
        moveToShootPath.add(new Waypoint(FieldDimensions.centerRedSideFarBallPos, medOptions));
        moveToShootPath.add(new Waypoint(FieldDimensions.midShootPos, medOptions));

        Vector2d backUpTargetPosStart = FieldDimensions.portPos.sub(startPose.getPosition());
        Vector2d backUpTargetPosMid = FieldDimensions.portPos.sub(FieldDimensions.midShootPos);




        runAction(new WaitAction(startDelaySec));

        AimShooterAction aimShooterAction1 = new AimShooterAction(backUpTargetPosStart.angle());
        runAction(aimShooterAction1);
        Vector2d targetPos = aimShooterAction1.getSensedTargetPos();
        if(targetPos != null){
            runAction(new SpeedUpShooterAction(targetPos.length()));
        } else {
            runAction(new SpeedUpShooterAction(backUpTargetPosStart.length()));
        }
        runAction(new FeedBallsAction(4));
        shooter.setShooterRPM(0.0);
        shooter.setTurretDeg(0.0);

        //Collecting Center Balls
        runAction(new PathFollowerAction(startToMid));
        runAction(new IntakeAction());
        runAction(new PathFollowerAction(ballCollectionPath));

        List<Action> actions = new ArrayList<Action>();
        actions.add(new SpeedUpShooterAction(backUpTargetPosMid.length()));
        actions.add(new )

        ParallelAction prepareToShoot = new ParallelAction(actions);
        runAction(prepareToShoot);
        runAction(new PathFollowerAction(moveToShootPath));









        runAction(new WaitAction(startDelaySec));


    }

}