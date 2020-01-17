package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Drive;

public class TimedDriveStraightAction implements Action {

    private double power;
    private double targetTime;
    private double startTime;

    private boolean finished = false;
    private Drive mDrive = Drive.getInstance();


    public TimedDriveStraightAction(double power, double timeSeconds){
        this.power = power;
        this.targetTime = timeSeconds;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        mDrive.setOpenLoop(new DriveCommand(power, power));
    }

    @Override
    public void update() {
        mDrive.setOpenLoop(new DriveCommand(power, power));
        if(Timer.getFPGATimestamp()-startTime >= targetTime){
            finished = true;
            System.out.println("Finished");
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(new DriveCommand(0,0));
    }

    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
	    }
    };
	
    public DataLogger getLogger() { return logger; }
}