package frc.robot;

public class OperationalMode
{
	// singleton class
    private static OperationalMode instance = null;
    public static OperationalMode getInstance() 
    { 
        if (instance == null) {
            instance = new OperationalMode();
        }
        return instance;
    }    

    enum OperationalModeEnum 
    {
    	DISABLED(0), AUTONOMOUS(1), TELEOP(2), TEST(3);
    	
    	public int val;
    	
    	private OperationalModeEnum (int _val) {this.val = _val;}
    }

    public OperationalModeEnum opMode = OperationalModeEnum.DISABLED;

    public void set(OperationalModeEnum _mode) { opMode = _mode; }
    public OperationalModeEnum get() { return opMode; }
}
