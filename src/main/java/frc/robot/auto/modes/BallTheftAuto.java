package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.AimShooterAction;
import frc.robot.auto.actions.FeedBallsAction;
import frc.robot.auto.actions.SpeedUpShooterAction;
import frc.robot.lib.util.Vector2d;
import frc.robot.subsystems.Shooter;

public class BallTheftAuto extends AutoModeBase {
    public BallTheftAuto(){}
    private Vector2d targetPos;
    private Shooter shooter = Shooter.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException {





        AimShooterAction aimAction = new AimShooterAction(45);
        runAction(aimAction);
        targetPos = aimAction.getSensedTargetPos();
        if(targetPos != null){
            runAction(new SpeedUpShooterAction(targetPos.length()));
            runAction(new FeedBallsAction(3));
            shooter.setShooterRPM(0.0);
            
        }
        

    }
}