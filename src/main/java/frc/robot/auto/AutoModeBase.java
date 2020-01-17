package frc.robot.auto;



import frc.robot.Constants;
import frc.robot.auto.actions.Action;
import frc.robot.lib.util.DataLogController;
import frc.robot.lib.util.Pose;

import edu.wpi.first.wpilibj.Timer;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This
 * is implemented in auto.modes (which are routines that do actions).
 */
public abstract class AutoModeBase
{
    protected double updatePeriod = Constants.kLoopDt;
    protected boolean active = false;
    protected Pose initialPose = new Pose();
    
    static DataLogController autoLogger = DataLogController.getAutoLogController();
    
    protected abstract void routine() throws AutoModeEndedException;

    public void run() 
    {
        active = true;
        try 
        {
            routine();
        } 
        catch (AutoModeEndedException e) 
        {
            System.out.println("Auto mode done, ended early");
            return;
        }
        done();
        System.out.println("Auto mode done");
    }

    public void done() 
    {
        active = false;
    }

    public void stop() 
    {
        active = false;
    }

    public boolean isActive() 
    {
        return active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException 
    {
        if (!isActive()) 
        {
            throw new AutoModeEndedException();
        }
        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException 
    {
        autoLogger.deregister();						// remove previous action loggers from registry
        autoLogger.register(action.getLogger());
        autoLogger.setOutputMode(true, true);
        
        action.start();
        while (isActiveWithThrow() && !action.isFinished()) 
        {
        	double currTime = Timer.getFPGATimestamp();
        	double nextTime = Timer.getFPGATimestamp() + updatePeriod;
        	
            action.update();
            autoLogger.log();

        	currTime = Timer.getFPGATimestamp();
            long waitTime = (long) ((nextTime-currTime) * 1000.0);	// attempt to run thread every updatePeriod seconds
            waitTime = Math.max(waitTime, 0);						// avoid negative waits
            try
            {
                Thread.sleep(waitTime);
            } 
            catch (InterruptedException e) 
            {
                e.printStackTrace();
            }
        }
        action.done();
        autoLogger.log();	// capture one last log
    }

    public Pose getInitialPose()
    {
    	return initialPose;	// default implementation
    }
    
    
}
