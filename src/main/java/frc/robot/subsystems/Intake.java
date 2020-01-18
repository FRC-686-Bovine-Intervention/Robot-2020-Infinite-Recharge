//initialize intake; turn on the motors-forward or reverse;

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.lib.joystick.DriverControlsBase;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.loops.Loop;


public class Intake extends Subsystem implements Loop
{
    public VictorSPX intakeMotor;

	// singleton class
	private static Intake instance = null;
	public static Intake getInstance() 
	{ 
		if (instance == null) {
			instance = new Intake();
		}
		return instance;
    }


    private int kSlotId = 0;
    private double kF = 0, kP = 0.5, kI = 0, kD = 1;
    private int kAllowableError = 10;
    private static final double extendedTime = 1;
    private double startTime;
    private boolean timerStarted = false;


    public Intake(){
        intakeMotor = new VictorSPX(Constants.kIntakeTalonId);

       // Factory default hardware to prevent unexpected behavior
        intakeMotor.configFactoryDefault();
        intakeMotor.setInverted(true);
        
        // intakeMotor.selectProfileSlot(kSlotId, Constants.kTalonPidIdx); 
        // intakeMotor.config_kF(kSlotId, kF, Constants.kTalonTimeoutMs); 
        // intakeMotor.config_kP(kSlotId, kP, Constants.kTalonTimeoutMs); 
        // intakeMotor.config_kI(kSlotId, kI, Constants.kTalonTimeoutMs); 
        // intakeMotor.config_kD(kSlotId, kD, Constants.kTalonTimeoutMs);
        // intakeMotor.configAllowableClosedloopError(kSlotId, kAllowableError, Constants.kTalonTimeoutMs);

    }
    

    //Loop functions
    @Override
    public void onStart() {
        stop();
    }

    @Override
    public void onLoop() {
        DriverControlsBase driverControls = SelectedDriverControls.getInstance().get();

        if (driverControls.getBoolean(DriverControlsEnum.INTAKE))
        {
            set(+Constants.kIntakeVoltage);
        }
        else if (driverControls.getBoolean(DriverControlsEnum.OUTTAKE))
        {
            set(-Constants.kIntakeVoltage);
        }
        else 
        {
            stop();
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
        set(0.0);
    }



    public void set(double speed)
    {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }
}