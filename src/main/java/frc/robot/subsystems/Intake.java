//initialize intake; turn on the motors-forward or reverse;

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public DoubleSolenoid mainSolenoids, secondarySolenoids;

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

    private double cPower = 0;


    public Intake(){
        intakeMotor = new VictorSPX(Constants.kIntakeTalonId);
        intakeMotor.configFactoryDefault();
        intakeMotor.setInverted(true);

        mainSolenoids = new DoubleSolenoid(Constants.kPCMID, Constants.kMainSolenoidFChannel, Constants.kMainSolenoidRChannel);
        secondarySolenoids = new DoubleSolenoid(Constants.kPCMID, Constants.kSecondarySolenoidFChannel, Constants.kSecondarySolenoidRChannel);

        SmartDashboard.putBoolean("Intake/Debug", false);
        SmartDashboard.putNumber("Intake/Debug/IntakePower", 0);
        SmartDashboard.putBoolean("Intake/Debug/MainSolenoids", false);
        SmartDashboard.putBoolean("Intake/Debug/SecondarySolenoids", false);
    }
    

    //Loop functions
    @Override
    public void onStart() {
        stop();
    }

    @Override
    public void onLoop() {
        if(!SmartDashboard.getBoolean("Lift/Debug", false)){
            DriverControlsBase driverControls = SelectedDriverControls.getInstance().get();

            if (driverControls.getBoolean(DriverControlsEnum.INTAKE_TOGGLE) && !toggleLastState){
                if(currentState == IntakeState.STORED){
                    //Toggle to floor
                    extendToFloor();
                    setPower(Constants.kIntakePower);
                } else if(currentState == IntakeState.GROUND){
                    //Toggle to stored
                    retract();
                }
            }
            else if (driverControls.getBoolean(DriverControlsEnum.INTAKE_STORED))
            {
                retract();
            }

            toggleLastState = driverControls.getBoolean(DriverControlsEnum.INTAKE_TOGGLE);
        } else {
            setPower(SmartDashboard.getNumber("Intake/Debug/IntakePower", 0));
            mainSolenoids.set(booleanToValue(SmartDashboard.getBoolean("Intake/Debug/MainSolenoids", false)));
            secondarySolenoids.set(booleanToValue(SmartDashboard.getBoolean("Intake/Debug/SecondarySolenoids", false)));
        }
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
        retract();
    }



    public void setPower(double percent)
    {
        intakeMotor.set(ControlMode.PercentOutput, percent);
        cPower = percent;
    }

    public void extendToFloor(){
        currentState = IntakeState.GROUND;
        mainSolenoids.set(DoubleSolenoid.Value.kForward);
        secondarySolenoids.set(DoubleSolenoid.Value.kForward);
    }

    public void retract(){
        currentState = IntakeState.STORED;
        setPower(0.0);
        mainSolenoids.set(DoubleSolenoid.Value.kReverse);
        secondarySolenoids.set(DoubleSolenoid.Value.kReverse);
    }

    public DoubleSolenoid.Value booleanToValue(boolean input){
        return input ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
    }

    public double getCurrentPower(){
        return cPower;
    }
}