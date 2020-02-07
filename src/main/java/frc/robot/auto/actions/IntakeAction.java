package frc.robot.auto.actions;

import frc.robot.subsystems.Shooter;
import frc.robot.lib.util.DataLogger;

public class IntakeAction implements Action {

    Shooter shooter = Shooter.getInstance();
    boolean finished = false;

    public IntakeAction() {}

    @Override
    public void start() {
        shooter.onLoop();
        finished = false;
    }

    @Override
    public void update() {
         finished = shooter.nearTarget(false);
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