package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.SmartDashboardInteractions;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.AimShooterAction;
import frc.robot.auto.actions.CalibrateAction;
import frc.robot.auto.actions.DriveStraightAction;
import frc.robot.auto.actions.DriveToTime;
import frc.robot.auto.actions.FeedBallsAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.SpeedUpShooterAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.sensors.Pigeon;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.subsystems.Shooter;

public class SumoAuto extends AutoModeBase {

    public SumoAuto(){}

    Shooter shooter = Shooter.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Hello2!!! \n Hello~~~~~");
        SmartDashboardInteractions  smartDashboard = SmartDashboardInteractions.getInstance();
        Pose startPose = smartDashboard.getSelectedStartPose();
        double startDelaySec = smartDashboard.getStartDelay();
        Vector2d backUpTargetPos = FieldDimensions.portPos.sub(startPose.getPosition());


        runAction(new WaitAction(startDelaySec));
        
        AimShooterAction aimShooterAction = new AimShooterAction();
        runAction(aimShooterAction);
        Vector2d targetPos = aimShooterAction.getSensedTargetPos();
        if(targetPos != null){
            runAction(new SpeedUpShooterAction(targetPos.length()));
        } else {
            runAction(new SpeedUpShooterAction(backUpTargetPos.length()));
            //runAction(new SpeedUpShooterAction(backUpTargetPos.length()));
        }
        runAction(new FeedBallsAction(3));
        shooter.setShooterRPM(0.0);

        
        List<Action> driveAndCalibrate = new ArrayList<Action>();
        driveAndCalibrate.add(new CalibrateAction());
        driveAndCalibrate.add(new DriveToTime(0.25, 1) ); //Drive off line and push teammate
        ParallelAction driveAndCalibrateAction = new ParallelAction(driveAndCalibrate);
        runAction(driveAndCalibrateAction);
    }

}