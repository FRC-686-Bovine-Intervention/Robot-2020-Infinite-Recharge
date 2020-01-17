package frc.robot.loops;


import java.util.ArrayList;
import java.util.List;

import frc.robot.lib.util.CrashTrackingRunnable;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List
 * object. They are started when the robot powers up and stopped after the
 * match.
 */
public class LoopController 
{
    public final double kPeriod = Constants.kLoopDt;

    private boolean running_;

    private final Notifier notifier_;	// the Notifier will run the function runCrashTracked() with a period of kPeriod
    
    private final List<Loop> loops_;
    private final Object taskRunningLock_ = new Object();
    private double prev_time_ = 0;
	protected double dt_;
   
    
    private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() 
    {
        @Override
        public void runCrashTracked() 
        {
        	// lock during access to loop_ to avoid corruption from multiple threads
            synchronized (taskRunningLock_) 
            {
                if (running_) 
                {
                    double curr_time = Timer.getFPGATimestamp();
                    for (Loop loop : loops_) 
                    {
                        loop.onLoop();
                    }
                    dt_ = curr_time - prev_time_;
                    prev_time_ = curr_time;
                }
            }
        }
    };

    
    public LoopController() 
    {
        notifier_ = new Notifier(runnable_);
        running_ = false;
        loops_ = new ArrayList<>();
    }

    public synchronized void register(Loop loop) 
    {
    	// lock during access to loop_ to avoid corruption from multiple threads
        synchronized (taskRunningLock_) 
        {
            loops_.add(loop);
        }
    }

    public synchronized void start() 
    {
        if (!running_) 
        {
            System.out.println("Starting loops");
        	// lock during access to loop_ to avoid corruption from multiple threads
            synchronized (taskRunningLock_) 
            {
                prev_time_ = Timer.getFPGATimestamp();
                for (Loop loop : loops_) 
                {
//                    System.out.println("Starting " + loop);
                    loop.onStart();
                }
                running_ = true;
            }
            // avoiding watchdog errors
            notifier_.startPeriodic(kPeriod);
        }
    }

    public synchronized void run() 
    {
        if (!running_) 
        {
        	// lock during access to loop_ to avoid corruption from multiple threads
            synchronized (taskRunningLock_) 
            {
                for (Loop loop : loops_) 
                {
                    loop.onLoop();
                }
            }
        }
    }


    public synchronized void stop() 
    {
        if (running_) 
        {
            System.out.println("Stopping loops");
            notifier_.stop();
        	// lock during access to loop_ to avoid corruption from multiple threads
            synchronized (taskRunningLock_) 
            {
                running_ = false;
                for (Loop loop : loops_) 
                {
//                    System.out.println("Stopping " + loop);
                    loop.onStop();
                }
            }
        }
    }

}
