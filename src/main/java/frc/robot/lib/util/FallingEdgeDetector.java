package frc.robot.lib.util;

/**
 * Falling Edge Detector. 
 * <p>
 * Returns true only when input changes from true to false.
 */
public class FallingEdgeDetector
{
    private boolean lastValue = false;

    public boolean update(boolean newValue) 
    {
        boolean rv = (!newValue && lastValue);
        lastValue = newValue;
        return rv;
    }
}