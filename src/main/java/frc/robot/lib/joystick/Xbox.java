package frc.robot.lib.joystick;

import frc.robot.lib.util.DataLogger;

public class Xbox extends JoystickBase
{
   public static int kButtonA = 1;
   public static int kButtonB = 2;
   public static int kButtonX = 3;
   public static int kButtonY = 4;
   public static int kButtonLB = 5;
   public static int kButtonRB = 6;

   public static int kLStickXAxis = 0;
   public static int kLStickYAxis = 1;
   public static int kLTriggerAxis = 2;
   public static int kRTriggerAxis = 3;
   public static int kRStickXAxis = 4;
   public static int kRStickYAxis = 5;

   // constructor
   public Xbox(int _port)
   {
       super(_port);
   }

   public DataLogger getLogger() { return logger; }
    
   private final DataLogger logger = new DataLogger()
   {
       @Override
       public void log()
       {
           put("Xbox/LStickXAxis", getAxis(kLStickXAxis));
           put("Xbox/LStickYAxis", getAxis(kLStickYAxis));
           put("Xbox/LTriggerAxis", getAxis(kLTriggerAxis));
           put("Xbox/RTriggerAxis", getAxis(kRTriggerAxis));
           put("Xbox/RStickXAxis", getAxis(kRStickXAxis));
           put("Xbox/RStickYAxis", getAxis(kRStickYAxis));
           int buttons = 0;
           for (int button=1; button<=6; button++)
           {
               buttons |= (getButton(button) ? 1 : 0) << (button-1);
           }
           put("Xbox/buttons", buttons);
           put("Xbox/pov", getPOV());
       }
   };
}