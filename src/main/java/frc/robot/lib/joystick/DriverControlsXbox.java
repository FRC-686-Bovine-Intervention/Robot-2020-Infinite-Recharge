package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.joystick.SteeringLib.NonLinearityEnum;
import frc.robot.lib.util.DataLogger;



public class DriverControlsXbox extends DriverControlsBase
{
	// singleton class
    private static DriverControlsXbox instance = null;
    public static DriverControlsXbox getInstance() 
    { 
        if (instance == null) {
            instance = new DriverControlsXbox();
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
 
    public DriverControlsXbox() 
    {
        controller = new Xbox(kControllerPort);
        buttonBoard = new ButtonBoard(kButtonBoardPort);

        double throttleDeadband =     0.02;
        double turnDeadband =         0.02;
        NonLinearityEnum throttleNonLinearity = NonLinearityEnum.SQUARED;
        NonLinearityEnum turnNonLinearity =     NonLinearityEnum.SQUARED;

        steeringControls = new TmArcadeDriveSteering(controller, new DeadbandNonLinearity(throttleDeadband, turnDeadband, throttleNonLinearity, turnNonLinearity));
    }

    public DriveCommand getDriveCommand() 
    {
        DriveCommand drvCmd = steeringControls.getDriveCommand();
        drvCmd.setMotors(0.8*drvCmd.getLeftMotor(), 0.8*drvCmd.getRightMotor());
        return drvCmd;
    }


    public boolean getBoolean( DriverControlsEnum _control ) 
    {
        switch (_control)
        {
            case SHOOT:                         return controller.getAxisAsButton(Xbox.kRTriggerAxis);
            case DRIVE_ASSIST:                  return controller.getButton(Xbox.kButtonLB);
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
