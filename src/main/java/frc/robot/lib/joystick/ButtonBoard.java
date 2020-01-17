package frc.robot.lib.joystick;

import frc.robot.lib.util.DataLogger;

public class ButtonBoard extends JoystickBase
{
    public static int kButtonA =        1;
    public static int kButtonB =        2;
    public static int kButtonX =        3;
    public static int kButtonY =        4;
    public static int kButtonLB =       5;
    public static int kButtonRB =       6;
    public static int kButtonShare =    7;
    public static int kButtonOptions =  8;
    public static int kButtonSL =       9;
    public static int kButtonSR =       10;

    public ButtonBoard(int _port) 
    {
        super(_port);
    }

    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            int buttons = 0;
            for (int button=1; button<=10; button++)
            {
                buttons |= (getButton(button) ? 1 : 0) << (button-1);
            }
            put("ButtonBoard/buttons", buttons);
        }
    };    
}
