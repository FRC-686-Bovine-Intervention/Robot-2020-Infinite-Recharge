package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.joystick.SteeringLib.NonLinearityEnum;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Drive;


public class DriverControlsThrustmasterDouble extends DriverControlsBase
{
	// singleton class
    private static DriverControlsThrustmasterRight instance = null;
    public static DriverControlsThrustmasterRight getInstance() 
    { 
        if (instance == null) {
            instance = new DriverControlsThrustmasterRight();
        }
        return instance;
    }
    
    
    // Joystick Port Constants
    public static int kControllerLeftPort = 0;
    public static int kButtonBoardPort = 1;
    public static int kControllerRightPort = 2;

    public static JoystickBase controllerLeft;
    public static JoystickBase controllerRight;
    public static JoystickBase buttonBoard;


    public static SteeringBase steeringControls;

    // button board constants
    // public static int kCargoIntakeRocketButton =    ButtonBoard.kButtonB;
 
    public DriverControlsThrustmasterDouble() 
    {
        controllerLeft = new Thrustmaster(kControllerLeftPort);
        controllerRight = new Thrustmaster(kControllerRightPort);
        buttonBoard = new ButtonBoard(kButtonBoardPort);

        double throttleDeadband =     0.02;
        double turnDeadband =         0.02;
        NonLinearityEnum throttleNonLinearity = NonLinearityEnum.SQUARED;
        NonLinearityEnum turnNonLinearity =     NonLinearityEnum.SQUARED;

        steeringControls = new TmArcadeDriveSteering(controllerRight, new DeadbandNonLinearity(throttleDeadband, turnDeadband, throttleNonLinearity, turnNonLinearity));
    }

    public DriveCommand getDriveCommand()
    {
        return steeringControls.getDriveCommand(); 
    }


    public boolean getBoolean( DriverControlsEnum _control ) 
    {
        // intake when: driving forward or turning, when outtake button is not pressed
        //              (so no intake when driving backwards or stopped)
        DriveCommand driveCmd = Drive.getInstance().getCommand();

        switch (_control)
        {
            case SHOOT:                         return controllerLeft.getButton(Thrustmaster.kTriggerButton);
            case DRIVE_ASSIST:                  return false;
            case INTAKE_TOGGLE:                 return controllerLeft.getButton(Thrustmaster.kBottomThumbButton);
            case INTAKE_STORED:                 return buttonBoard.getButton(ButtonBoard.kButtonRB);
            case LOCK_LIFT:                     return buttonBoard.getButton(ButtonBoard.kButtonY);
            case UNLOCK_LIFT:                   return buttonBoard.getButton(ButtonBoard.kButtonX);
            case TOGGLE_PTO:                    return controllerRight.getButton(Thrustmaster.kBottomButton6);
            case CALIBRATE:                     return buttonBoard.getButton(ButtonBoard.kButtonA);
            case RESET:                         return buttonBoard.getButton(ButtonBoard.kButtonB);
            case QUICK_TURN:                    return false;
            default:                            return false;
        }
    }

    public double getAxis( DriverAxisEnum _axis ) 
    {
        switch (_axis)
        {
            case SHOOTER_SPEED_CORRECTION:      return controllerLeft.getAxis(Thrustmaster.kSliderAxis);
            default:                            return 0.0;
        }
    }


    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            if (controllerLeft != null)         { controllerLeft.getLogger().log(); }
            if (controllerRight != null)         { controllerRight.getLogger().log(); }
            if (buttonBoard != null)        { buttonBoard.getLogger().log(); }
            if (steeringControls != null)   { steeringControls.getLogger().log(); }
        }
    };        
}
