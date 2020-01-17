package frc.robot.auto.actions;

import frc.robot.subsystems.Shooter;
import frc.robot.lib.util.DataLogger;

public class SetShooterSpeedAction implements Action {

    Shooter shooter = Shooter.getInstance();
    boolean finished = false;
    Shooter.GoalEnum goal = Shooter.GoalEnum.HIGH_GOAL;

    public SetShooterSpeedAction(Shooter.GoalEnum _goal){
        goal = _goal;
    }

    @Override
    public void start() {
        shooter.setTarget(goal);        
        finished = false;
    }

    @Override
    public void update() {
        finished = shooter.nearTarget(true);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
    }

    // @Override
    // public DataLogger getLogger() {
    //     return null;
    // }
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
	    }
    };
	
	@Override
	public DataLogger getLogger() { return logger; }

}