package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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

    private Solenoid PTOLeftSolenoid, PTORightSolenoid, lockSolenoid;
    private DigitalInput liftRetractionSensor;

    private boolean PTOLastState = false;
    private boolean liftRetracted = false;

    private boolean solenoidDriveActiveVal = true; //What value the solenoid is at when drive is activated
    private boolean lockActiveVal = true; //The value activates the lock
    
    public enum PTOTansmissionState {
        DRIVE_ENABLED, LIFT_ENABLED
    }

    public PTOTansmissionState cPTOState = PTOTansmissionState.DRIVE_ENABLED;


    public Lift(){
        PTOLeftSolenoid = new Solenoid(Constants.kPCMID, Constants.kPTOSolenoidChannel);
        lockSolenoid = new Solenoid(Constants.kPCMID, Constants.kLiftLockSolenoidChannel);
        liftRetractionSensor = new DigitalInput(Constants.kLiftSensorChannel);
    }

    @Override
    public void onStart() {
        stop();
    }

    @Override
    public void onLoop() {
        DriverControlsBase driverControls = SelectedDriverControls.getInstance().get();
		if(driverControls.getBoolean(DriverControlsEnum.TOGGLE_PTO) && !PTOLastState){
			if(cPTOState == PTOTansmissionState.DRIVE_ENABLED){
                shiftToLift();
            } else if(cPTOState == PTOTansmissionState.LIFT_ENABLED){
                shiftToDrive();
            }
        }
        PTOLastState = driverControls.getBoolean(DriverControlsEnum.TOGGLE_PTO);

        if(driverControls.getBoolean(DriverControlsEnum.LOCK_LIFT) && checkLift()) {
            lockLift();
        }

        if(driverControls.getBoolean(DriverControlsEnum.UNLOCK_LIFT)){
            unlockLift();
        }
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
        lockSolenoid.set(lockActiveVal);
    }

    public void unlockLift(){
        lockSolenoid.set(!lockActiveVal);
    }

    public void shiftToDrive(){
        PTOLeftSolenoid.set(solenoidDriveActiveVal);
        cPTOState = PTOTansmissionState.DRIVE_ENABLED;
    }

    public void shiftToLift(){
        PTOLeftSolenoid.set(!solenoidDriveActiveVal);
        cPTOState = PTOTansmissionState.LIFT_ENABLED;
    }


    public boolean checkLift(){
        return !liftRetractionSensor.get();
    }


    
    public PTOTansmissionState getPTOState(){
        return cPTOState;
    }

}