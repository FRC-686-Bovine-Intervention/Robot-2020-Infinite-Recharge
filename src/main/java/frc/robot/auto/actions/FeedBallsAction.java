package frc.robot.auto.actions;

import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.ConveyorBelt;

public class FeedBallsAction implements Action {

    private ConveyorBelt conveyorBelt = ConveyorBelt.getInstance();

    private int numberOfBalls; 
    private boolean finished;


    public FeedBallsAction(int numberOfBalls){
        this.numberOfBalls = numberOfBalls;
    }



    @Override
    public void start() {
        conveyorBelt.setPosition(numberOfBalls);
    }

    @Override
    public void update() {
        if(conveyorBelt.nearTarget()){
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
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