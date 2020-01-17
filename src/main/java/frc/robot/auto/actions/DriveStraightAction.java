package frc.robot.auto.actions;

import frc.robot.lib.util.DataLogger;
import frc.robot.command_status.DriveState;
import frc.robot.subsystems.*;

/**
 * DriveStraightAction drives the robot straight at a settable angle, distance,
 * and velocity. This action begins by setting the drive controller, and then
 * waits until the distance is reached.
 *
 * @see Action
 * @see Drive
 * @see Rotation2d
 */
public class DriveStraightAction implements Action {

    private double startingDistance;
    private double mWantedDistance;
    private double mVelocity;
    private double mHeadingDeg;
    private Drive mDrive = Drive.getInstance();
    private DriveState driveState = DriveState.getInstance();

    public DriveStraightAction(double distance, double velocity) {
        this(distance, velocity, 0);
    }

    public DriveStraightAction(double distance, double velocity, double headingDeg) {
        mWantedDistance = distance;
        mVelocity = velocity;
        mHeadingDeg = headingDeg;
    }

    @Override
    public void start() 
    {
        startingDistance = getCurrentDistance();
    }

    @Override
    public void update() 
    {
        mDrive.setVelocityHeadingSetpoint(mVelocity, mHeadingDeg);
        // mDrive.setVelocitySetpoint(mVelocity, mVelocity);
    }

    @Override
    public boolean isFinished() 
    {
    	System.out.printf("startingDistance=%7.3f, currDist=%7.3f, mWantedDistance=%7.3f\n", startingDistance, getCurrentDistance(), mWantedDistance);    	
    	
        boolean rv = false;
        if (mWantedDistance > 0) 
            rv = (getCurrentDistance() - startingDistance) >= mWantedDistance;
        else
            rv = (getCurrentDistance() - startingDistance) <= mWantedDistance;
        
        return rv;
    }

    @Override
    public void done()
    {
        mDrive.setVelocitySetpoint(0, 0);
    }

    private double getCurrentDistance() 
    {
        return (driveState.getLeftDistanceInches() + driveState.getRightDistanceInches()) / 2;
    }
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
    		put("AutoAction", "DriveStraight" );
			put("DriveCmd/talonMode", driveState.getTalonControlMode().toString() );
			put("DriveCmd/left", mDrive.getCommand().getLeftMotor() );
			put("DriveCmd/right", mDrive.getCommand().getRightMotor() );
    		put("DriveState/TalonControlMode", driveState.getTalonControlMode().toString() );
			put("DriveState/lSpeed", driveState.getLeftSpeedInchesPerSec() );
			put("DriveState/rSpeed", driveState.getRightSpeedInchesPerSec() );
    		put("DriveState/lDistance", driveState.getLeftDistanceInches() );
    		put("DriveState/rDistance", driveState.getRightDistanceInches() );
    		put("DriveState/Heading", driveState.getHeadingDeg() );
	    }
    };
	
    public DataLogger getLogger() { return logger; }
    
}
