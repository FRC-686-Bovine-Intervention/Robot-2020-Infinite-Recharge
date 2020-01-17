package frc.robot.auto.actions;

import java.util.ArrayList;
import java.util.List;

import frc.robot.lib.util.DataLogger;

/**
 * Executes one action at a time. Useful as a member of {@link ParallelAction}
 */
public class SeriesAction implements Action 
{

    private Action mCurAction;
    private final ArrayList<Action> mRemainingActions;

    public SeriesAction() 
    {
        mRemainingActions = new ArrayList<>();
        mCurAction = null;
    }

    public SeriesAction(List<Action> actions) 
    {
        mRemainingActions = new ArrayList<>(actions.size());
        mRemainingActions.addAll(actions);
        mCurAction = null;
    }

	public void add(List<Action> actions)
	{
		for (Action action : actions) 
		{
			mRemainingActions.add(action);
		}
	}
	
	public void add(Action action)
	{
		mRemainingActions.add(action);
	}

    @Override
    public boolean isFinished() 
    {
        return mRemainingActions.isEmpty() && mCurAction == null;
    }

    @Override
    public void start() 
    {
    }

    @Override
    public void update() 
    {
        if (mCurAction == null) 
        {
            if (mRemainingActions.isEmpty()) 
            {
                return;
            }
            mCurAction = mRemainingActions.remove(0);
            mCurAction.start();
        }
        mCurAction.update();
        if (mCurAction.isFinished()) 
        {
            mCurAction.done();
            mCurAction = null;
        }
    }

    @Override
    public void done() 
    {
    }
    
    
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
//            if (mCurAction == null) 
//            	mCurAction.getLogger().log();
	    }
    };
	
    public DataLogger getLogger() { return logger; }
    
}
