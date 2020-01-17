package frc.robot.lib.util;

/**
 * Rising Edge Detector. 
 * <p>
 * Returns true only when input changes from false to true.
 */
public class RisingEdgeDetector
{
    private boolean lastValue = false;
    private boolean edge = false;

    public boolean update(boolean newValue) 
    {
        edge = (newValue && !lastValue);
        lastValue = newValue;
        return edge;
    }

    public boolean get()
    {
        return edge;
    }
}