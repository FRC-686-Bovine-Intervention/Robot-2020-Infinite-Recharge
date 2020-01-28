package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.lib.joystick.DriverControlsBase;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.loops.Loop;

public class Lift extends Subsystem implements Loop {

    private static Lift instance = null;
    public static Lift getInstance(){
        if(instance == null){
            instance = new Lift();
        }
        return instance;
    }

    private Solenoid PTOSolenoid, lockSolenoid;

    private boolean PTOLastState = false;
	private boolean liftLockLastState = false;
	private boolean liftLocked = false;

	public enum LiftState {
		RETRACTED, EXTENDED, TRANSITIONING
    }
    public enum PTOTansmissionState {
        DRIVE_ENABLED, LIFT_ENABLED
    }

    public LiftState cLiftState = LiftState.RETRACTED;
    public PTOTansmissionState cPTOState = PTOTansmissionState.DRIVE_ENABLED;


    public Lift(){
        PTOSolenoid = new Solenoid(Constants.kPCMID, Constants.kPTOSolenoidChannel);
        lockSolenoid = new Solenoid(Constants.kPCMID, Constants.kLiftLockSolenoidChannel);
    }

    @Override
    public void onStart() {
        stop();
    }

    @Override
    public void onLoop() {
        DriverControlsBase driverControls = SelectedDriverControls.getInstance().get();
		if(driverControls.getBoolean(DriverControlsEnum.TOGGLE_PTO) && !PTOLastState){
			
        }
        
        PTOLastState = driverControls.getBoolean(DriverControlsEnum.TOGGLE_PTO);
		liftLockLastState = driverControls.getBoolean(DriverControlsEnum.LIFT_LOCK);
    }

    @Override
    public void onStop() {
        stop();
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }



    public void lockLift(){
        lockSolenoid.set(true);
    }

    public void unlockLift(){
        lockSolenoid.set(false);
    }

    public void shiftToDrive(){
        lockSolenoid.set(false);
    }

    public void shiftToLift(){
        lockSolenoid.set(true);
    }


}