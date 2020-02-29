package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.command_status.DriveCommand;
import frc.robot.lib.sensors.NavX;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Drive;

public class DriveToTime implements Action {
    private double time;
    private double power;
    private DriveCommand driveCommand;
    private Drive mDrive = Drive.getInstance();
    private double startTime;
    private boolean finished = false;
    
    public DriveToTime(double power, double timeSeconds) {
        this.time = timeSeconds;
        this.power = power;
    }

    @Override
    public void start() {
        driveCommand = new DriveCommand(power, power);
        mDrive.setOpenLoop(driveCommand);

        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        driveCommand.setMotors(power, power);
        mDrive.setOpenLoop(driveCommand);

        if(Timer.getFPGATimestamp()-startTime >= time){
            finished = true;
            mDrive.setOpenLoop(new DriveCommand(0,0));
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
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