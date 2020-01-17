package frc.robot.command_status;

import frc.robot.lib.util.DataLogger;

public class VisionStatus
{
	// singleton class
	private static VisionStatus instance = null;
	public static VisionStatus getInstance() 
	{ 
		if (instance == null) {
			instance = new VisionStatus();
		}
		return instance;
	}

	private double imageTimestamp;

	private double normalizedTargetX;
	private double normalizedTargetWidth;
	
	public synchronized void   setImageTimestamp( double _val ) { imageTimestamp = _val; }
	public synchronized double getImageTimestamp() { return imageTimestamp; }
	
	public synchronized void   setNormalizedTargetX( double _val ) { normalizedTargetX = _val; }
	public synchronized double getNormalizedTargetX() { return normalizedTargetX; }

	public synchronized void   setNormalizedTargetWidth( double _val ) { normalizedTargetWidth = _val; }
	public synchronized double getNormalizedTargetWidth() { return normalizedTargetWidth; }
	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        	synchronized (this)
        	{
 	    		put("VisionStatus/imageTimestamp", imageTimestamp );
	    		put("VisionStatus/normalizedTargetX", normalizedTargetX );
	    		put("VisionStatus/normalizedTargetWidth", normalizedTargetWidth );
        	}
        }
    };
    
    public DataLogger getLogger() { return logger; }
	
}
