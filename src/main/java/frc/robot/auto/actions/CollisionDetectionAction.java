package frc.robot.auto.actions;

import frc.robot.command_status.DriveState;
import frc.robot.lib.sensors.GyroBase;
import frc.robot.lib.sensors.NavX;
import frc.robot.lib.util.DataLogger;

/**
 * Action for following a path defined by a Path object.
 * 
 * Serially configures a PathFollower object to follow each path 
 */
public class CollisionDetectionAction implements Action 
{
	public static NavX gyro;
	private double lastAccelX;
	private double lastAccelY;
	double jerkX = 0.0;
	double jerkY = 0.0;
	double lMotorCurrent = 0.0;
	double rMotorCurrent = 0.0;
    
    public static double kCollisionVel 			= 24;
    public static double kCollisionAccelTime = 0.5;	// sec to reach max velocity
    public static double kCollisionAccel 		= kCollisionVel / kCollisionAccelTime;
    public static double kCollisionJerkThreshold 	= 0.9;		// maximum JerkY was 0.9 for a 24 inch/sec collision into wall (<0.1 when driving normal)
    public static double kCollisionCurrentThreshold = 20;		// threshold to detect stall current
    

    public CollisionDetectionAction() 
    {
    	gyro = NavX.getInstance();
    }

    @Override
    public void start() 
    {
		System.out.println("Starting CollisionDetectionAction");
    	lastAccelX = gyro.getWorldLinearAccelerationX();
    	lastAccelY = gyro.getWorldLinearAccelerationY();
    }


    @Override
    public void update() 
    {
    	// do nothing -- just waiting for a collision
    }	
	
	
    @Override
    public boolean isFinished() 
    {
        double accelX = gyro.getWorldLinearAccelerationX();
        double accelY = gyro.getWorldLinearAccelerationY();
        jerkX = accelX - lastAccelX;
        jerkY = accelY - lastAccelY;
        lastAccelX = accelX;
        lastAccelY = accelY;

        //System.out.println(this.toString());  
       
        boolean collisionDetected = false;
        if ( ( Math.abs(jerkX) > kCollisionJerkThreshold ) || ( Math.abs(jerkY) > kCollisionJerkThreshold) )
        	collisionDetected = true;
        
        lMotorCurrent = DriveState.getInstance().getLeftMotorCurrent();
        rMotorCurrent = DriveState.getInstance().getRightMotorCurrent();
        if ( ( lMotorCurrent > kCollisionCurrentThreshold ) || ( rMotorCurrent > kCollisionCurrentThreshold) )
        	collisionDetected = true;
        
    	return collisionDetected;
    }

    @Override
    public void done() 
    {
		System.out.println("Finished CollisionDetectionAction");
        System.out.println(this.toString());  
		
		// cleanup code, if any
    }

    public String toString()
    {
    	return String.format("Collision Detection -- JerkX: % 5.3f, JerkY: % 5.3f, JerkThresh: %4.1f, lMotorCurrent: % 5.3f, rMotorCurrent: % 5.3f, CurrentThresh: %4.1f, ", jerkX, jerkY, kCollisionJerkThreshold, lMotorCurrent, rMotorCurrent, kCollisionCurrentThreshold);
    }
    
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
    		put("AutoAction/AutoAction", "CollisionDetectionAction" );
   			put("CollisionDetectionAction/JerkX", jerkX );
   			put("CollisionDetectionAction/JerkY", jerkY );
   			put("CollisionDetectionAction/JerkThresh", kCollisionJerkThreshold );
   			put("CollisionDetectionAction/lMotorCurrent", jerkX );
   			put("CollisionDetectionAction/rMotorCurrent", jerkY );
   			put("CollisionDetectionAction/CurrentThresh", kCollisionCurrentThreshold );
        }
    };
     
    public DataLogger getLogger() { return logger; }
}
