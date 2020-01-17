package frc.robot.lib.util;

import java.util.Optional;

/*
 * A PathSegment consists of a start and end position
 * and various properties for the robot while on that segment
 * (the speed, lookahead distance, whether to use vision, etc.)
 */
public class PathSegment 
{
    protected Vector2d start;
    protected Vector2d end;
    protected Vector2d startToEnd; 		// pre-computed for efficiency
    protected double   length; 			// pre-computed for efficiency

    protected Options options;
    
    static public class Options
    {
	    protected double   maxSpeed;		// speed along this segment (always positive)
	    protected double   maxAccel;		// maximum acceleration along this segment (always positive)
	    protected double   lookaheadDist;	// lookahead distance along this segment (use smaller for tighter turns) 
	    protected boolean  visionEnable;	// whether vision is enabled for this segment
	    									// vision will take over from path following when the target is identified
        protected Optional<String> marker;

	    // constructor
	    public Options(double _maxSpeed, double _maxAccel, double _lookaheadDist, boolean _visionEnable)
	    {
	        maxSpeed = _maxSpeed;
	        maxAccel = _maxAccel;
	        lookaheadDist = _lookaheadDist;
	        visionEnable = _visionEnable;
          	marker = Optional.empty();
	    }

	    public Options(double _maxSpeed, double _maxAccel, double _lookaheadDist, boolean _visionEnable, String _marker)
	    {
	        maxSpeed = _maxSpeed;
	        maxAccel = _maxAccel;
	        lookaheadDist = _lookaheadDist;
	        visionEnable = _visionEnable;
          	marker = Optional.of(_marker);
	    }

	    // copy constructor
	    public Options(Options _options)
	    {
	    	this(_options.maxSpeed, _options.maxAccel, _options.lookaheadDist, _options.visionEnable);
	    }
	    
	    public double   getMaxSpeed()   		{ return maxSpeed; }		
	    public double   getMaxAccel()   		{ return maxAccel; }		
	    public double   getLookaheadDist()  { return lookaheadDist; }
	    public boolean  getVisionEnable()   { return visionEnable; }
	    public Optional<String> getMarker() { return marker; }
	    
	    public String toString()
	    {
	    	return String.format("MaxSpeed: %5.1f, MaxAccel: %5.1f: LookaheadDist: %4.1f, VisionEnable: %b", maxSpeed, maxAccel, lookaheadDist, visionEnable);
	    }
	    
    };
	    
	    
    public PathSegment(Vector2d _start, Vector2d _end, Options _options) 
    {
        end = _end;
        updateStart(_start);
        options = new Options(_options);
    }

    public void updateStart(Vector2d newStart)
    {
        start = newStart;		
        startToEnd = end.sub(start);
        length = startToEnd.length();
    }

    // copy constructor
    public PathSegment(PathSegment _seg)
    {
    	start 		= new Vector2d(_seg.start);
    	end   	   	= new Vector2d(_seg.end);
    	startToEnd 	= new Vector2d(_seg.startToEnd);
    	length		= _seg.length;
    	options		= new Options(_seg.options);
    }
    
    public Vector2d getStart()   { return new Vector2d(start); }
    public Vector2d getEnd()     { return new Vector2d(end); }
    public double   getLength()  { return length; }
    public Options getOptions() { return new Options(options); }
    
    public Util.ClosestPointOnSegment getClosestPoint(Vector2d _position)
    {
    	return Util.getClosestPointOnSegment(start, end, _position);  
    }
    

    public Vector2d interpolate(double index)
    {
    	return start.interpolate(end, index);
    }

    // check alignment of vector (start, _that) to segment (start, end)
    public double dotProduct(Vector2d _that)
    {
        Vector2d startToThat = _that.sub(start);
        return startToEnd.dot(startToThat);
    }
    
    public String toString()
    {
    	return "Start: " + start.toString() + ", End: " + end.toString() + ", Options: " + options.toString();
    }
    
}
