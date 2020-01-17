package frc.robot.auto.actions;

import frc.robot.command_status.RobotState;
import frc.robot.lib.util.DataLogger;


public class CrossXYAction implements Action 
{
	public RobotState robotState = RobotState.getInstance();
	double threshold;
	double diff;
	double prevDiff;
	boolean xFlag = true;
	
    public CrossXYAction(char _xy, double _threshold) 
    {
    	xFlag = ((_xy == 'x') || (_xy == 'X'));

    	threshold = _threshold;
    	diff = 0;
    	prevDiff = 0;
    	
		if (xFlag)
    		System.out.println("CrossXYAction X Threshold: " + threshold);
		else
    		System.out.println("CrossXYAction Y Threshold: " + threshold);
    }

    @Override
    public void start() 
    {
    	if (xFlag)
    		diff = (robotState.getLatestFieldToVehicle().getX() - threshold);
    	else
    		diff = (robotState.getLatestFieldToVehicle().getY() - threshold);
    	
    	prevDiff = diff;
    }


    @Override
    public void update() 
    {
    	// do nothing -- just waiting for a collision
    }	
	
	
    @Override
    public boolean isFinished() 
    {
    	if (xFlag)
    		diff = (robotState.getLatestFieldToVehicle().getX() - threshold);
    	else
    		diff = (robotState.getLatestFieldToVehicle().getY() - threshold);

    	
    	// we are done when we cross the threshold line
    	// (so the sign of the difference will change)
    	boolean finished = (Math.signum(diff) != Math.signum(prevDiff));
    	
    	if (finished)
    	{
    		if (xFlag)
        		System.out.println("Crossed X Threshold: " + robotState.getLatestFieldToVehicle().getX());
    		else
        		System.out.println("Crossed Y Threshold: " + robotState.getLatestFieldToVehicle().getY());
		}

    	prevDiff = diff;
    	return finished;
    }

    @Override
    public void done() 
    {
		// cleanup code, if any
    }

	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        }
    };
     
    public DataLogger getLogger() { return logger; }
}
