//initialize intake; turn on the motors-forward or reverse;

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.lib.joystick.DriverControlsBase;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.loops.Loop;


public class Intake extends Subsystem implements Loop
{
	// singleton class
	private static Intake instance = null;
	public static Intake getInstance() 
	{ 
		if (instance == null) {
			instance = new Intake();
		}
		return instance;
    }

    public VictorSPX intakeMotor;
    public Solenoid mainSolenoid, secondarySolenoid;


    private int kSlotId = 0;
    public static final int kSlotIdxSpeed = 0;
    private double kF = 0, kP = 0.5, kI = 0, kD = 1;
    private int kAllowableError = 10;
    private double startTime;
    private boolean timerStarted = false;

    public static final int kPeakCurrentLimit = 30;
    public static final int kPeakCurrentDuration = 200;
    public static final int kContinuousCurrentLimit = 20;

    public static enum IntakeState {
        STORED, PLAYER_STATION, GROUND
    }
    private IntakeState currentState = IntakeState.STORED;
    private boolean toggleLastState = false;


    public Intake(){
        intakeMotor = new VictorSPX(Constants.kIntakeTalonId);
        intakeMotor.configFactoryDefault();
        intakeMotor.setInverted(true);
        mainSolenoid = new Solenoid(Constants.kPCMID, Constants.kMainSolenoidChannel);
        secondarySolenoid = new Solenoid(Constants.kPCMID, Constants.kSecondarySolenoidChannel);
    }
    

    //Loop functions
    @Override
    public void onStart() {
        stop();
    }

    @Override
    public void onLoop() {
        DriverControlsBase driverControls = SelectedDriverControls.getInstance().get();

        if (driverControls.getBoolean(DriverControlsEnum.INTAKE_GROUND) && !toggleLastState)
        {
            if(currentState == IntakeState.STORED){
                //Toggle to floor
                extendToFloor();
            } else if(currentState == IntakeState.GROUND){
                //Toggle to stored
                retract();
            } else if(currentState == IntakeState.PLAYER_STATION){
                //Jump to ground position
                extendToPlayerStation();
            }
        }
        else if(driverControls.getBoolean(DriverControlsEnum.INTAKE_PLAYERSTATION))
        {
            extendToPlayerStation();
        } 
        else if (driverControls.getBoolean(DriverControlsEnum.INTAKE_STORED))
        {
            retract();
        }

        toggleLastState = driverControls.getBoolean(DriverControlsEnum.INTAKE_GROUND);
    }



    @Override
    public void onStop() {
        stop();
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {
        setPower(0.0);
        retract();
    }



    public void setPower(double percent)
    {
        intakeMotor.set(ControlMode.PercentOutput, percent);
    }

    public void extendToFloor(){
        currentState = IntakeState.GROUND;
        setPower(+Constants.kIntakeVoltage);
        mainSolenoid.set(true);
        secondarySolenoid.set(true);
    }

    public void extendToPlayerStation(){
        currentState = IntakeState.PLAYER_STATION;
        setPower(+Constants.kIntakeVoltage);
        mainSolenoid.set(true);
        secondarySolenoid.set(false);
    }

    public void retract(){
        currentState = IntakeState.STORED;
        setPower(0);
        mainSolenoid.set(false);
        secondarySolenoid.set(false);
    }
}