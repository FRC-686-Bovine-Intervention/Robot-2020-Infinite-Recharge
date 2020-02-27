package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private Solenoid PTOSolenoid1, PTOSolenoid2, lockSolenoid1, lockSolenoid2;

    private boolean PTOLastState = false;

    private boolean driveActiveVal = true; //What value the solenoid is at when drive is activated
    private boolean lockActiveVal = true; //The value activates the lock
    
    public enum PTOTansmissionState {
        DRIVE_ENABLED, LIFT_ENABLED
    }

    public PTOTansmissionState cPTOState = PTOTansmissionState.DRIVE_ENABLED;


    public Lift(){
        PTOSolenoid1 = new Solenoid(Constants.kPCMID, Constants.kPTOSolenoid1Channel);
        PTOSolenoid2 = new Solenoid(Constants.kPCMID, Constants.kPTOSolenoid2Channel);
        lockSolenoid1 = new Solenoid(Constants.kPCMID, Constants.kLiftLockSolenoid1Channel);
        lockSolenoid2 = new Solenoid(Constants.kPCMID, Constants.kLiftLockSolenoid2Channel);

        lockLift();
        shiftToDrive();

        SmartDashboard.putBoolean("Lift/Debug", false);
        SmartDashboard.putBoolean("Lift/Debug/PTOSolenoids", false);
        SmartDashboard.putBoolean("Lift/Debug/LockSolenoids", false);
    }

    @Override
    public void onStart() {
        stop();
    }

    @Override
    public void onLoop() {
        if(!SmartDashboard.getBoolean("Lift/Debug", false)){
            DriverControlsBase driverControls = SelectedDriverControls.getInstance().get();
            if(driverControls.getBoolean(DriverControlsEnum.TOGGLE_PTO) && !PTOLastState){
                if(cPTOState == PTOTansmissionState.DRIVE_ENABLED){
                    shiftToLift();
                    unlockLift();
                } else if(cPTOState == PTOTansmissionState.LIFT_ENABLED){
                    shiftToDrive();
                    lockLift();
                }
            }
            PTOLastState = driverControls.getBoolean(DriverControlsEnum.TOGGLE_PTO);

            if(driverControls.getBoolean(DriverControlsEnum.LOCK_LIFT)) {
                lockLift();
            } else if(driverControls.getBoolean(DriverControlsEnum.UNLOCK_LIFT)){
                unlockLift();
            }
        } else {
            PTOSolenoid1.set(SmartDashboard.getBoolean("Lift/Debug/PTOSolenoids", false));
            PTOSolenoid2.set(SmartDashboard.getBoolean("Lift/Debug/PTOSolenoids", false));
            lockSolenoid1.set(SmartDashboard.getBoolean("Lift/Debug/LockSolenoids", false));
            lockSolenoid2.set(SmartDashboard.getBoolean("Lift/Debug/LockSolenoids", false));
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
        lockSolenoid1.set(lockActiveVal);
        lockSolenoid2.set(lockActiveVal);
    }

    public void unlockLift(){
        lockSolenoid1.set(!lockActiveVal);
        lockSolenoid2.set(!lockActiveVal);
    }

    public void shiftToDrive(){
        PTOSolenoid1.set(driveActiveVal);
        PTOSolenoid2.set(driveActiveVal);
        cPTOState = PTOTansmissionState.DRIVE_ENABLED;
    }

    public void shiftToLift(){
        PTOSolenoid1.set(!driveActiveVal);
        PTOSolenoid2.set(!driveActiveVal);
        cPTOState = PTOTansmissionState.LIFT_ENABLED;
    }

    
    public PTOTansmissionState getPTOState(){
        return cPTOState;
    }

}