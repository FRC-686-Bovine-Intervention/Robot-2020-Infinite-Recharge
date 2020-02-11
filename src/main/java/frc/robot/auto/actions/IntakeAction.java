package frc.robot.auto.actions;

import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Intake;

public class IntakeAction implements Action {

    Intake intake = Intake.getInstance();
    boolean finished = false;
    double speed = 1;

    public IntakeAction() {}

    @Override
    public void start() {
        intake.setPower(speed);
        intake.extendToFloor();
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