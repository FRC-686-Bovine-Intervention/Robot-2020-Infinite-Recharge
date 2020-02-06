package frc.robot.auto.actions;


import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Shooter;

public class turretResetAction implements Action {

    Shooter shooter = Shooter.getInstance();
    boolean finished = false;
    double turretAngle = 0;
   

    public turretResetAction() {}
    
    @Override
    public void start() {
        shooter.onLoop();
        shooter.setTurretDeg(turretAngle);
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