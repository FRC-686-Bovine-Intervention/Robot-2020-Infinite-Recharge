package frc.robot.auto.actions;

import frc.robot.lib.util.DataLogger;

/**
 * Composite action, running all sub-actions at the same time All actions are
 * started then updated until all actions report being done.
 * 
 * The entire set of actions will be declared done if
 * 		1: the entire list of actions report done
 * 		2: any of the interrupting actions report done 
 * 
 * @param A
 *            List of Action objects
 */
public class InterruptableAction implements Action 
{

	private final Action mInterruptingAction;
	private final Action mAction;
    
    public InterruptableAction(Action interruptingAction, Action actions) 
    {
    	mInterruptingAction = interruptingAction;
    	mAction = actions;
    }

    @Override
    public void start() 
    {
    	mInterruptingAction.start();
    	mAction.start();
    }
    
    @Override
    public void update() 
    {
    	mInterruptingAction.update();
    	mAction.update();
    }

    @Override
    public boolean isFinished() 
    {
    	boolean finished = (mInterruptingAction.isFinished()) || (mAction.isFinished()); 

		if (finished)
			System.out.println("InterruptableAction Finished");
    	
        return finished;
    }

    @Override
    public void done() 
    {
    	mInterruptingAction.done();
    	mAction.done();
    }

	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        	mAction.getLogger().log();
        	mInterruptingAction.getLogger().log();
	    }
    };
	
    public DataLogger getLogger() { return logger; }
    
}
