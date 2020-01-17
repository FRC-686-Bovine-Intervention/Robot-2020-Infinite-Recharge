package frc.robot.lib.sensors;

/**
 * An abstract class for a simple gyro interface.
 */
public abstract class GyroBase 
{
 	// Returns robot heading in degrees.  
	// 0 is along the x-axis, angle increases as robot turns to the left
	// angle is continuous, and does not reset to zero when passing +/- 360 degrees
    public abstract double getHeadingDeg();

	public void zeroSensor()
	{
		// no code
	}
}
