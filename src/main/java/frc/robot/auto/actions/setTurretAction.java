package frc.robot.auto.actions;


import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Shooter;

public class setTurretAction implements Action {

    Shooter shooter = Shooter.getInstance();
    boolean finished = false;
    double angle = shooter.getTargetDisplacement().angle();
   

    public setTurretAction() {}
    
    @Override
    public void start() {
        shooter.onLoop();
        shooter.setTurretAbsDeg(angle);
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