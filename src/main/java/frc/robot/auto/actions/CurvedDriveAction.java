package frc.robot.auto.actions;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.command_status.DriveCommand;
import frc.robot.lib.sensors.NavX;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Kinematics;
import frc.robot.lib.util.Kinematics.WheelSpeed;
import frc.robot.subsystems.Drive;

public class CurvedDriveAction implements Action {

    private double power;
    private double radius;
    private double targetAngle;
    private double curvature;
    private WheelSpeed speed;
    private DriveCommand driveCommand;

    public static final double allowableError = 10;

    private boolean finished = false;
    private Drive mDrive = Drive.getInstance();


    public CurvedDriveAction(double power, double radius, double targetAngle){
        this.power = power;
        this.radius = radius;
        this.targetAngle = targetAngle;
    }

    @Override
    public void start() {
        curvature = 1.0/radius;
        speed = Kinematics.inverseKinematicsFromSpeedCurvature(power, curvature);
        driveCommand = new DriveCommand(speed.left, speed.right);
        driveCommand.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void update() {
        mDrive.setOpenLoop(driveCommand);

        if(Math.abs(NavX.getInstance().getHeadingDeg()-targetAngle) <= allowableError){
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
        driveCommand.setNeutralMode(NeutralMode.Coast);
        driveCommand.setMotors(0, 0);
        mDrive.setOpenLoop(driveCommand);
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