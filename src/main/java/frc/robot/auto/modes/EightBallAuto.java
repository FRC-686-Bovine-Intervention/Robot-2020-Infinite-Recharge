package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.SmartDashboardInteractions;
import frc.robot.auto.*;
import frc.robot.auto.actions.*;
import frc.robot.lib.util.*;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.subsystems.Shooter;
import frc.robot.lib.sensors.Pigeon;

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



        //============================
        //Execution:
        //============================

        runAction(new WaitAction(startDelaySec)); //Delay should probably be zero due to length of this mode. Might move this elsewhere

        AimShooterAction aimShooterAction1 = new AimShooterAction();
        runAction(aimShooterAction1);
        Vector2d targetPos = aimShooterAction1.getSensedTargetPos();
        if(targetPos != null){
            runAction(new SpeedUpShooterAction(targetPos.length()));
        } else {
            runAction(new SpeedUpShooterAction(backUpTargetPosStart.length()));
        }
        runAction(new FeedBallsAction(4));
        shooter.setShooterRPM(0.0);

        //Moving To Center
        List<Action> prepareToCollectActions = new ArrayList<Action>();
        prepareToCollectActions.add(new PathFollowerAction(startToMid));
        prepareToCollectActions.add(new IntakeAction());
        prepareToCollectActions.add(new CalibrateAction()); //Added so we are ready for teleop and next shot
        runAction(new ParallelAction(prepareToCollectActions));

        //Collecting
        runAction(new PathFollowerAction(ballCollectionPath));

        //Preemptively getting ready to shoot while moving into position
        List<Action> actions = new ArrayList<Action>();
        actions.add(new SpeedUpShooterAction(backUpTargetPosMid.length()));
        actions.add(new IntakeStopAction());
        actions.add(new PathFollowerAction(moveToShootPath));
        ParallelAction prepareToShoot = new ParallelAction(actions);
        runAction(prepareToShoot);

        //Finding Target
        AimShooterAction aimShooterAction2 = new AimShooterAction(backUpTargetPosMid.angle()-pigeon.getHeadingDeg());
        runAction(aimShooterAction2);
        targetPos = null; //Resetting so that the last taken measurement won't affect the following code
        targetPos = aimShooterAction2.getSensedTargetPos();

        //Determining if the speed should be changed and shooting:
        if(targetPos != null){
            runAction(new SpeedUpShooterAction(targetPos.length()));
        }
        //Shooter was already spun up to speed, so no need to handle target loss if it were to occur
        runAction(new FeedBallsAction(6));
        shooter.setShooterRPM(0.0);
        shooter.setTurretAbsDeg(0.0);

        //Done!
    }

}