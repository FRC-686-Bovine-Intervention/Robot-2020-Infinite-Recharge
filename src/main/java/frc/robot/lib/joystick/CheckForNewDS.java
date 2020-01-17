package frc.robot.lib.joystick;


public class CheckForNewDS 
{
	// singleton class
    private static CheckForNewDS instance = null;
    public static CheckForNewDS getInstance() 
    { 
        if (instance == null) {
            instance = new CheckForNewDS();
        }
        return instance;
    }    
    boolean newData = false;

    public void reset()
    {
        newData = true;
    }

    public boolean check()
    {
        boolean retVal = newData;
        newData = false;
        return retVal;
    }
}