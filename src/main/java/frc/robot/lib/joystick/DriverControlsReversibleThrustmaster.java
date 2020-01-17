package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.joystick.SteeringLib.NonLinearityEnum;
import frc.robot.lib.util.DataLogger;



public class DriverControlsReversibleThrustmaster extends ReversibleDriverControlsBase
{
	// singleton class
    private static DriverControlsReversibleThrustmaster instance = null;
    public static DriverControlsReversibleThrustmaster getInstance() 
    { 
        if (instance == null) {
            instance = new DriverControlsReversibleThrustmaster();
        }
        return instance;
    }
    
    
    // Joystick Port Constants
    public static int kLeftStickPort = 0;
    public static int kRightStickPort = 1;
    public static int kButtonBoardPort = 2;

    public static JoystickBase lStick;
    public static JoystickBase rStick;
    public static JoystickBase buttonBoard;

    public static ReversibleSteeringBase steeringControls;

    // button board constants
    public static int kCargoIntakeRocketButton =    ButtonBoard.kButtonB;
    public static int kCargoIntakeCargoShipButton = ButtonBoard.kButtonA;
    public static int kDefenseButton =              ButtonBoard.kButtonRB;
    public static int kClimbingStartButton =        ButtonBoard.kButtonLB;
    public static int kClimbingExtendButton =       ButtonBoard.kButtonX;
    public static int kClimbingRetractButton =      ButtonBoard.kButtonY;
    public static int kEmergencyZeroingAxis =       ButtonBoard.kButtonSR;

    public DriverControlsReversibleThrustmaster() 
    {
        lStick = new Thrustmaster(kLeftStickPort);
        rStick = new Thrustmaster(kRightStickPort);
        buttonBoard = new ButtonBoard(kButtonBoardPort);

        double throttleDeadband =     0.02;
        double turnDeadband =         0.02;
        NonLinearityEnum throttleNonLinearity = NonLinearityEnum.SQUARED;
        NonLinearityEnum turnNonLinearity =     NonLinearityEnum.SQUARED;

        steeringControls = new TmReversibleArcadeDriveSteering(lStick, rStick, new DeadbandNonLinearity(throttleDeadband, turnDeadband, throttleNonLinearity, turnNonLinearity));
    }

    public DriveCommand getDriveCommand() 
    {
        return steeringControls.getDriveCommand(); 
    }


    public boolean getBoolean( DriverControlsEnum _control ) 
    {
        switch (_control)
        {
            case INTAKE:                        return false;
            case OUTTAKE:                       return false;
            case SHOOT:                         return lStick.getButton(Thrustmaster.kTriggerButton) || lStick.getButton(Thrustmaster.kBottomThumbButton);
            case TARGET_LOW:                    return lStick.getButton(Thrustmaster.kBottomThumbButton) && !lStick.getButton(Thrustmaster.kTriggerButton);
            case QUICK_TURN:                    return false;
            default:                            return false;
        }
    }

    public double getAxis( DriverAxisEnum _axis ) 
    {
        switch (_axis)
        {
            case SHOOTER_SPEED_CORRECTION:      return lStick.getAxis(Thrustmaster.kSliderAxis);
            default:                            return 0.0;
        }
    }

    public boolean getDrivingCargo()
    {
        return steeringControls.usingLeftStick();
    }

    public boolean joystickActive()
    {
        return steeringControls.joystickActive();
    }



    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            if (lStick != null)             { lStick.getLogger().log(); }
            if (rStick != null)             { rStick.getLogger().log(); }
            if (buttonBoard != null)        { buttonBoard.getLogger().log(); }
            if (steeringControls != null)   { steeringControls.getLogger().log(); }
        }
    };    

}
