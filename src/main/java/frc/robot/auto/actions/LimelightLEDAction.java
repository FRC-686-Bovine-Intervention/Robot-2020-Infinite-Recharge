package frc.robot.auto.actions;

import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.sensors.Limelight.LedMode;
import frc.robot.lib.util.DataLogger;

public class LimelightLEDAction implements Action {

    Limelight camera = Limelight.getInstance();
    boolean finished = false;
    LedMode LEDMode = LedMode.kOn;

    public LimelightLEDAction(LedMode _LEDMode){
        LEDMode = _LEDMode;
    }

    @Override
    public void start() {
        camera.setLEDMode(LEDMode);
        finished = false;
    }

    @Override
    public void update() {
        finished = true;
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