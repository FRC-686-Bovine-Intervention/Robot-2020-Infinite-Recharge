package frc.robot.auto;

import frc.robot.lib.util.CrashTrackingRunnable;

/**
 * This class selects, runs, and (if necessary) stops a specified autonomous mode
 */
public class AutoModeExecuter 
{
    private AutoModeBase autoMode;
    private Thread autoThread = null;
    
    public void setAutoMode(AutoModeBase _autoMode) 
    {
        autoMode = _autoMode;
    }

    public AutoModeBase getAutoMode() 
    {
        return autoMode;
    }

    public void start() 
    {

        if (autoThread == null) 
        {
            autoThread = new Thread(new CrashTrackingRunnable() 
            {
                @Override
                public void runCrashTracked() 
                {
                    if (autoMode != null) 
                    {
                        autoMode.run();
                    }
                }
            });
            autoThread.start();
        }

    }

    public void stop() 
    {
        if (autoMode != null) 
        {
            autoMode.stop();
        }
        autoThread = null;
    }

}
