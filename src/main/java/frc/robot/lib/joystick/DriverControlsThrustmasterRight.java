package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.joystick.SteeringLib.NonLinearityEnum;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Drive;


public class DriverControlsThrustmasterRight extends DriverControlsBase
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
    public static int kControllerPort = 0;
    public static int kButtonBoardPort = 1;

    public static JoystickBase controller;
    public static JoystickBase buttonBoard;


    public static SteeringBase steeringControls;

    // button board constants
    // public static int kCargoIntakeRocketButton =    ButtonBoard.kButtonB;
 
    public DriverControlsThrustmasterRight() 
    {
        controller = new Thrustmaster(kControllerPort);
        buttonBoard = new ButtonBoard(kButtonBoardPort);

        double throttleDeadband =     0.02;
        double turnDeadband =         0.02;
        NonLinearityEnum throttleNonLinearity = NonLinearityEnum.SQUARED;
        NonLinearityEnum turnNonLinearity =     NonLinearityEnum.SQUARED;

        steeringControls = new TmArcadeDriveSteering(controller, new DeadbandNonLinearity(throttleDeadband, turnDeadband, throttleNonLinearity, turnNonLinearity));
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
            case SHOOT:                         return controller.getButton(Thrustmaster.kTriggerButton);
            case DRIVE_ASSIST:                  return false;
            case INTAKE_TOGGLE:                 return controller.getButton(Thrustmaster.kBottomThumbButton);
            case INTAKE_STORED:                 return buttonBoard.getButton(ButtonBoard.kButtonRB);
            case LOCK_LIFT:                     return buttonBoard.getButton(ButtonBoard.kButtonY);
            case UNLOCK_LIFT:                   return buttonBoard.getButton(ButtonBoard.kButtonX);
            case TOGGLE_PTO:                    return controller.getButton(Thrustmaster.kBottomButton6);
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
            case SHOOTER_SPEED_CORRECTION:      return controller.getAxis(Thrustmaster.kSliderAxis);
            default:                            return 0.0;
        }
    }


    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            if (controller != null)         { controller.getLogger().log(); }
            if (buttonBoard != null)        { buttonBoard.getLogger().log(); }
            if (steeringControls != null)   { steeringControls.getLogger().log(); }
        }
    };        
}
