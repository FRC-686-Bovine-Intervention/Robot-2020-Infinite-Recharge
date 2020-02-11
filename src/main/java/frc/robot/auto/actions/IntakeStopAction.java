package frc.robot.auto.actions;

import frc.robot.subsystems.Intake;
import frc.robot.lib.util.DataLogger;

public class IntakeStopAction implements Action {

    Intake intake = Intake.getInstance();
    boolean finished = false;
    double speed = 0;

    public IntakeStopAction() {}

    @Override
    public void start() {
        intake.setPower(speed);
        intake.retract();
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