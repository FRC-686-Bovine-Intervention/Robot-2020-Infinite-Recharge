package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.SmartDashboardInteractions;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.AimShooterAction;
import frc.robot.auto.actions.CalibrateAction;
import frc.robot.auto.actions.FeedBallsAction;
import frc.robot.auto.actions.IntakeAction;
import frc.robot.auto.actions.IntakeStopAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.SpeedUpShooterAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.subsystems.Shooter;

public class BallTheftAuto extends AutoModeBase {
    public BallTheftAuto(){}
    private Vector2d targetPos;
    private Shooter shooter = Shooter.getInstance();
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
        Path startToOpponentTrench = new Path();
        startToOpponentTrench.add(new Waypoint(startPosition, fastOptions));
        startToOpponentTrench.add(new Waypoint(FieldDimensions.opponentTrenchApproachPos, fastOptions));

        Path approachingOpponentCellsPath = new Path();
        approachingOpponentCellsPath.add(new Waypoint(FieldDimensions.opponentTrenchApproachPos, slowOptions));
        approachingOpponentCellsPath.add(new Waypoint(FieldDimensions.opponentTrenchBallPos, slowOptions));

        //Begin movements
        runAction(new WaitAction(startDelaySec));
        
        List<Action> moveAndCalibrate = new ArrayList<Action>();
        moveAndCalibrate.add(new PathFollowerAction(startToOpponentTrench));
        moveAndCalibrate.add(new CalibrateAction());
        ParallelAction moveAndCalibrateAction = new ParallelAction(moveAndCalibrate);

        runAction(moveAndCalibrateAction);
        runAction(new IntakeAction());
        runAction(new PathFollowerAction(approachingOpponentCellsPath));
        runAction(new IntakeStopAction());

        AimShooterAction aimAction = new AimShooterAction(FieldDimensions.angleOppTrenchToPort);
        runAction(aimAction);
        targetPos = aimAction.getSensedTargetPos();
        if(targetPos != null){
            runAction(new SpeedUpShooterAction(targetPos.length()));
            runAction(new FeedBallsAction(3));
            shooter.setShooterRPM(0.0);
        } else {
            //Go off estimates taken beforehand
            runAction(new SpeedUpShooterAction(FieldDimensions.distOppTrenchToPort));
            runAction(new FeedBallsAction(3));
            shooter.setShooterRPM(0.0);
        }
        shooter.setTurretAbsDeg(0);
    }
}