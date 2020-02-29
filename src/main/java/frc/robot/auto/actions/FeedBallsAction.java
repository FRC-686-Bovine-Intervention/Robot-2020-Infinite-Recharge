package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.ConveyorBelt;

public class FeedBallsAction implements Action {

    private ConveyorBelt conveyorBelt = ConveyorBelt.getInstance();

    private int numberOfBalls; 
    private boolean finished;
    private static final double timePerBall = 1;
    private double startTime = 0;


    public FeedBallsAction(int numberOfBalls){
        this.numberOfBalls = numberOfBalls;
    }



    @Override
    public void start() {
        conveyorBelt.feed();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        double elapsedTime = Timer.getFPGATimestamp()-startTime;
        if(numberOfBalls*timePerBall <=elapsedTime){
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
        conveyorBelt.stopAll();
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