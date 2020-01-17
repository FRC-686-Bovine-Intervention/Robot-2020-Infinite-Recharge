package frc.robot.lib.util;


public class Vector2d implements Interpolable<Vector2d>
{
	protected double x;
	protected double y;
	
	public static final double TWO_PI = 2 * Math.PI;
	
	// constructors
	public Vector2d()
	{
		x = 0;
		y = 0;
	}

	public Vector2d(double _x, double _y)
	{
		x = _x;
		y = _y;
	}
	
	public Vector2d(Vector2d _v)
	{
		x = _v.x;
		y = _v.y;
	}
	
	// sets/gets
	public void setX(double x) { this.x = x; }
	public void setY(double y) { this.y = y; }
	
    public double getX() { return x; }
	public double getY() { return y; }

	
	// radians<-->degrees conversions
	static public final double radiansToDegrees = 180 / Math.PI;
	static public final double radDeg = radiansToDegrees;
	static public final double degreesToRadians = Math.PI / 180;
	static public final double degRad = degreesToRadians;
    
	
	
	// arithmetic
	public Vector2d add(Vector2d _v) { return new Vector2d(x + _v.x, y + _v.y); }
    public Vector2d sub(Vector2d _v) { return new Vector2d(x - _v.x, y - _v.y); }
    public Vector2d neg() { return new Vector2d(-x, -y); }

    // absolute value (length) of vector
    public double abs()
    {
    	return Math.hypot(x, y);
    }
    
    // angle from origin to this
    public double angle()
    {
    	return Math.atan2(y, x);
    }

    // angle from this to _v
    public double angle(Vector2d _v)
    {
    	double dx = _v.x - this.x;
    	double dy = _v.y - this.y;
    	return Math.atan2(dy, dx);
    }

    // vector cross product
	public double cross(Vector2d _v)
	{
		return (this.x * _v.y - this.y * _v.x);
	}
    
    // distance from this to _v
    public double distance(Vector2d _v)
    {
    	double dx = _v.x - this.x;
    	double dy = _v.y - this.y;
    	return Math.hypot(dx, dy);
    }

    public double distanceSqr(Vector2d _v)
    {
    	double dx = _v.x - this.x;
    	double dy = _v.y - this.y;
    	return (dx*dx + dy*dy);
    }
    
    // vector dot product
	public double dot(Vector2d _v)
	{
		return (this.x * _v.x + this.y * _v.y);
	}
    
    /*
     *  perform exponential filtering on position
     *  alpha is the filtering coefficient, 0<alpha<<1
     *  result will converge 63% in 1/alpha timesteps
     *                       86% in 2/alpha timesteps
     *                       95% in 3/alpha timesteps
     */
    public Vector2d expAverage(Vector2d _v, double _alpha)
    {
    	// exponential averaging
    	// u = (1-a)*u + a*v
    	double ax = (1-_alpha)*x + _alpha*_v.x;
    	double ay = (1-_alpha)*y + _alpha*_v.y;
    	return new Vector2d(ax, ay);
    }
    
    // absolute value (length) of vector
    public double length()
    {
    	return Math.hypot(x, y);
    }
    
    // squared length of vector
    public double lengthSqr()
    {
    	return (x*x + y*y);
    }
    
    static public Vector2d magnitudeAngle(double _mag, double _angle)
    {
    	return new Vector2d(_mag*Math.cos(_angle), _mag*Math.sin(_angle));

    }

    // normalize angle to within [-pi, pi)
    public static double normalizeAngle(double _theta)
    {
    	double angle = _theta;
    	angle += Math.PI;
    	angle -= TWO_PI * Math.floor(angle / TWO_PI);
    	angle -= Math.PI;
     	return angle;
    }
    
    // normalize angle to within [-180.0, 180.0)
    public static double normalizeAngleDeg(double _thetaDeg)
    {
    	double angleDeg = _thetaDeg;
    	angleDeg += 180.0;
    	angleDeg -= 360.0 * Math.floor(angleDeg / 360.0);
    	angleDeg -= 180.0;
    	return angleDeg;
    }
    
    
	// Rotates Vector about a specified point by the given angle
	public Vector2d rotateAboutPoint(double _angle, Vector2d _centerOfRotation)
	{
		Vector2d rv = new Vector2d(this);
		rv.sub(_centerOfRotation);		// move center of rotation to origin
		rv.rotate(_angle);				// rotate about origin
		rv.add(_centerOfRotation);		// move center of rotation back
		return rv;
	}
    
	// Rotates Vector about the origin by the given angle
	public Vector2d rotate(double _angle)
	{
		double cos = Math.cos(_angle);
		double sin = Math.sin(_angle);

		double x = this.x * cos - this.y * sin;
		double y = this.x * sin + this.y * cos;
		return new Vector2d(x,y);
	}
    
	// Rotates Vector by the given angle
	public Vector2d rotateDeg(double _angleDeg)
	{
		return this.rotate(_angleDeg * degreesToRadians);
	}
    
	// return complex conjugate
	public Vector2d conj()
	{
		return new Vector2d(x, -y);
	}
	

    // linearly interpolate between this (for u=0) and that (for u=1)
    @Override
    public Vector2d interpolate(Vector2d that, double _u)
    {
    	double u = _u;
        if (u < 0)
        	u = 0;
        if (u > 1)
        	u = 1;
        
        double x = this.x + u*(that.x - this.x);
        double y = this.y + u*(that.y - this.y);
		return new Vector2d(x,y);
    }
    
	
	@Override
	public String toString ()
	{
		return String.format("(% 7.3f, % 7.3f)", x, y);
	}
    
}
