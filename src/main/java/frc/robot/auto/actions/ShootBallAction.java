package frc.robot.auto.actions;

import frc.robot.Agitator;
import frc.robot.lib.util.DataLogger;

public class ShootBallAction implements Action {

    Agitator agitator = Agitator.getInstance();
    boolean finished = false;
    int balls = 0;

    public ShootBallAction(int _balls)
    {
        balls = _balls;
    }

    @Override
    public void start() {
        agitator.shootBalls(balls);
        finished = false;
    }

    @Override
    public void update() {
        finished = agitator.nearTarget();
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