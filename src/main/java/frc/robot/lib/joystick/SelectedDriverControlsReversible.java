package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;

/**
 * Stores the DriverControls selected in SmartDashboard.
 * To be updated at startup and TeleopInit()
 */
public class SelectedDriverControlsReversible
{
	private static SelectedDriverControlsReversible instance = null;

	public static SelectedDriverControlsReversible getInstance() {
		if (instance == null) {
			instance = new SelectedDriverControlsReversible();
		}
		return instance;
    }    

    ReversibleDriverControlsBase driverControls = DriverControlsReversibleThrustmaster.getInstance();    // default selection

    public SelectedDriverControlsReversible()
    {
        setDriverControls( DriverControlsReversibleThrustmaster.getInstance() );    // default selection
    }

    public void setDriverControls(ReversibleDriverControlsBase _driverControls)
    {
        driverControls = _driverControls;
    }

    public DriverControlsBase get()
    {
        return driverControls;
    }

    // pass-thru gets
    public DriveCommand getDriveCommand() { return driverControls.getDriveCommand(); }
    public boolean getBoolean( DriverControlsEnum _control ) { return driverControls.getBoolean(_control); }
    public boolean getDrivingCargo() { return driverControls.getDrivingCargo(); }
    public boolean joystickActive() { return driverControls.joystickActive(); }




    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            driverControls.getLogger().log();
        }
    };        
}