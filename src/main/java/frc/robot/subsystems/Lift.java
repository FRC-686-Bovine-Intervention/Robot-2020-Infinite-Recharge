package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.joystick.DriverControlsBase;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.loops.Loop;

public class Lift extends Subsystem implements Loop {

    private static Lift instance = null;
    public static Lift getInstance(){
        if(instance == null){
            instance = new Lift();
        }
        return instance;
    }

    private Solenoid PTOSolenoids, lockSolenoids;

    private boolean PTOLastState = false;

    private boolean driveActiveVal = false; //What value the solenoid is at when drive is activated
    private boolean lockActiveVal = false; //The value activates the lock

    private RisingEdgeDetector PTOToggleEdge = new RisingEdgeDetector();
    
    public enum PTOTansmissionState {
        DRIVE_ENABLED, LIFT_ENABLED
    }

    public PTOTansmissionState cPTOState = PTOTansmissionState.DRIVE_ENABLED;


    public Lift(){
        PTOSolenoids = new Solenoid(Constants.kPCMID, Constants.kPTOSolenoidChannel);
        lockSolenoids = new Solenoid(Constants.kPCMID, Constants.kLiftLockSolenoidChannel);

        lockLift();
        shiftToDrive();

        SmartDashboard.putBoolean("Lift/Debug", false);
        SmartDashboard.putBoolean("Lift/Debug/PTOSolenoids", false);
        SmartDashboard.putBoolean("Lift/Debug/LockSolenoids", false);
    }

    @Override
    public void onStart() {
        stop();
        shiftToDrive();
        lockLift();
    }

    @Override
    public void onLoop() {
        if(!SmartDashboard.getBoolean("Lift/Debug", false)){
            DriverControlsBase driverControls = SelectedDriverControls.getInstance().get();

            if(PTOToggleEdge.update(driverControls.getBoolean(DriverControlsEnum.TOGGLE_PTO))){
                if(cPTOState == PTOTansmissionState.DRIVE_ENABLED){
                    shiftToLift();
                    unlockLift();
                } else if(cPTOState == PTOTansmissionState.LIFT_ENABLED){
                    shiftToDrive();
                    lockLift();
                }
            }

            if(driverControls.getBoolean(DriverControlsEnum.LOCK_LIFT)) {
                lockLift();
            } else if(driverControls.getBoolean(DriverControlsEnum.UNLOCK_LIFT)){
                unlockLift();
            }
        } else {
            PTOSolenoids.set(SmartDashboard.getBoolean("Lift/Debug/PTOSolenoids", false));
            lockSolenoids.set(SmartDashboard.getBoolean("Lift/Debug/LockSolenoids", false));
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
        lockSolenoids.set(lockActiveVal);
    }

    public void unlockLift(){
        lockSolenoids.set(!lockActiveVal);
    }

    public void shiftToDrive(){
        PTOSolenoids.set(driveActiveVal);
        cPTOState = PTOTansmissionState.DRIVE_ENABLED;
    }

    public void shiftToLift(){
        PTOSolenoids.set(!driveActiveVal);
        cPTOState = PTOTansmissionState.LIFT_ENABLED;
    }

    
    public PTOTansmissionState getPTOState(){
        return cPTOState;
    }

}