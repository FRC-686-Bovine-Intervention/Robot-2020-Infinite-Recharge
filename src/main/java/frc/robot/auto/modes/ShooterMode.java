package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetShooterSpeedAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.subsystems.Shooter;

public class ShooterMode extends AutoModeBase {

    public ShooterMode() 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetShooterSpeedAction(Shooter.GoalEnum.HIGH_GOAL));
        runAction(new WaitAction(5));
        runAction(new SetShooterSpeedAction(Shooter.GoalEnum.LOW_GOAL));
        runAction(new WaitAction(5));
    }
}