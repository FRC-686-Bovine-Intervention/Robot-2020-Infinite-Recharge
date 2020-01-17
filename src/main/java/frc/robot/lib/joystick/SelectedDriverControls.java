package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;

/**
 * Stores the DriverControls selected in SmartDashboard.
 * To be updated at startup and TeleopInit()
 */
public class SelectedDriverControls
{
	private static SelectedDriverControls instance = null;

	public static SelectedDriverControls getInstance() {
		if (instance == null) {
			instance = new SelectedDriverControls();
		}
		return instance;
    }    

    DriverControlsBase driverControls = DriverControlsThrustmaster.getInstance();    // default selection

    public SelectedDriverControls()
    {
        setDriverControls( DriverControlsThrustmaster.getInstance() );    // default selection
    }

    public void setDriverControls(DriverControlsBase _driverControls)
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
    public double getAxis(DriverAxisEnum _axis ) { return driverControls.getAxis(_axis); }
    public DataLogger getLogger() { return driverControls.getLogger(); }

}