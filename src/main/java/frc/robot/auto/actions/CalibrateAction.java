package frc.robot.auto.actions;

import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Shooter;

public class CalibrateAction implements Action {
    Shooter shooter = Shooter.getInstance();

    boolean finished = false;

    public CalibrateAction() {
    }

    @Override
    public void start() {
        //Do nothing
    }

    @Override
    public void update() {
        shooter.calibrate();
        if(shooter.allCalibrated){
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
        shooter.resetForCalibration();
    }

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