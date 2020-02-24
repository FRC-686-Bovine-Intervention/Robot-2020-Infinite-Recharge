package frc.robot.auto.modes;

import frc.robot.SmartDashboardInteractions;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.AimShooterAction;
import frc.robot.auto.actions.DriveStraightAction;
import frc.robot.auto.actions.FeedBallsAction;
import frc.robot.auto.actions.SpeedUpShooterAction;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.subsystems.Shooter;

public class SumoAuto extends AutoModeBase {

    public SumoAuto(){}

    Shooter shooter = Shooter.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException {
        SmartDashboardInteractions  smartDashboard = SmartDashboardInteractions.getInstance();
        Pose startPose = smartDashboard.getSelectedStartPose();
        Vector2d backUpTargetPos = FieldDimensions.portPos.sub(startPose.getPosition());


        AimShooterAction aimShooterAction = new AimShooterAction(backUpTargetPos.angle());
        runAction(aimShooterAction);
        Vector2d targetPos = aimShooterAction.getSensedTargetPos();
        if(targetPos != null){
            runAction(new SpeedUpShooterAction(targetPos.length()));
        } else {
            runAction(new SpeedUpShooterAction(backUpTargetPos.length()));
        }
        runAction(new FeedBallsAction(4));
        shooter.setShooterRPM(0.0);
        shooter.setTurretAbsDeg(0.0);


        //Drive off line and push teammate (distance, velocity, heading)
        runAction(new DriveStraightAction(-48,-24,startPose.getHeadingDeg()));
    


    }

}