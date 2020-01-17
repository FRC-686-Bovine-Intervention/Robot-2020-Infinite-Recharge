package frc.robot.lib.joystick;

import frc.robot.lib.util.DataLogger;

public class Thrustmaster extends JoystickBase
{
    public static int kXAxis =              0;
    public static int kYAxis =              1;
    public static int kZRotateAxis =        2;
    public static int kSliderAxis =         3;
    
    public static int kTriggerButton =      1;
    public static int kBottomThumbButton =  2;
    public static int kLeftThumbButton =    3;
    public static int kRightThumbButton =   4;

    // counting from inner (or thumb) side
    public static int kTopButton1 =		    5;         
    public static int kTopButton2 =		    6;
    public static int kTopButton3 =		    7;
    public static int kBottomButton3 =		8;
    public static int kBottomButton2 =		9;
    public static int kBottomButton1 =		10;
    public static int kTopButton6 =		    11;
    public static int kTopButton5 =		    12;
    public static int kTopButton4 =		    13;
    public static int kBottomButton4 =		14;
    public static int kBottomButton5 =		15;
    public static int kBottomButton6 =      16;
    
    public int port;

    // constructor
    public Thrustmaster(int _port)
    {
        super(_port);
        port = _port;
    }

    public DataLogger getLogger() { return logger; }
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            String name = "Thrustmaster" + port + "/";
            put(name + "xAxis", getAxis(kXAxis));
            put(name + "yAxis", getAxis(kYAxis));
            put(name + "zAxis", getAxis(kZRotateAxis));
            put(name + "sliderAxis", getAxis(kSliderAxis));
            int buttons = 0;
            for (int button=1; button<=16; button++)
            {
                buttons |= (getButton(button) ? 1 : 0) << (button-1);
            }
            put(name + "buttons", buttons);
            put(name + "pov", getPOV());
        }
    };
}