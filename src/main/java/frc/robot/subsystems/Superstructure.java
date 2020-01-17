package frc.robot.subsystems;

import frc.robot.lib.util.DataLogger;

public class Superstructure extends Subsystem {
	
 	// singleton class
	 private static Superstructure instance = null;
	 public static Superstructure getInstance() 
	 { 
		 if (instance == null) {
			 instance = new Superstructure();
		 }
		 return instance;
	 }
	
	private Superstructure()
	{
	}
	
	public void disable()
	{
	}
	
	public void enable()
	{
	}

	
	  
	@Override
	public void stop()
	{
	}

	@Override
	public void zeroSensors() {}

	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        }
        
    };
    
    public DataLogger getLogger() { return logger; }
	
	
}
