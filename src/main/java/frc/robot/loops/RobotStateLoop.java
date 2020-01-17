package frc.robot.loops;


import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;

import edu.wpi.first.wpilibj.Timer;

/**
 * Periodically estimates the state of the robot using the robot's distance
 * traveled (compares two waypoints), gyroscope orientation, and velocity, among
 * various other factors. Similar to a car's odometer.
 */
public class RobotStateLoop implements Loop 
{
 	// singleton class
	 private static RobotStateLoop instance = null;
	 public static RobotStateLoop getInstance() 
	 { 
		 if (instance == null) {
			 instance = new RobotStateLoop();
		 }
		 return instance;
	 }

    RobotState robotState;
    DriveState driveState;
    
    RobotStateLoop() 
    {
        robotState = RobotState.getInstance();
        driveState = DriveState.getInstance();
    }
    


    @Override
    public void onStart() 
    {
    	robotState.setPrevEncoderDistance(driveState.getLeftDistanceInches(), driveState.getRightDistanceInches());
    }

    @Override
    public void onLoop() 
    {
    	// the following DriveState elements are set during DriveLoop, called just previous to RobotStateLoop,
    	// and in the same LoopController thread
    	
        double time      = Timer.getFPGATimestamp();
        double lDistance = driveState.getLeftDistanceInches();
        double rDistance = driveState.getRightDistanceInches();
        double lSpeed    = driveState.getLeftSpeedInchesPerSec();
        double rSpeed    = driveState.getRightSpeedInchesPerSec(); 
        double gyroAngle = driveState.getHeading();

        robotState.generateOdometryFromSensors(time, lDistance, rDistance, lSpeed, rSpeed, gyroAngle);
    }

    @Override
    public void onStop() 
    {
        // no-op
    }

}
