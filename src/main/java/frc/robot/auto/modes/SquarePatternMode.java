package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Vector2d;
import frc.robot.loops.DriveLoop;

/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class SquarePatternMode extends AutoModeBase {

	Path path;
    public SquarePatternMode() 
    {
 
    }
    
    private void init(){
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Square Pattern");

    	init();
        
        double maxVel = 24; //DriveLoop.kPathFollowingMaxVel;
        double lookAhead = 30; // DriveLoop.kPathFollowingLookahead
    	PathSegment.Options options = new PathSegment.Options(maxVel, DriveLoop.kPathFollowingMaxAccel, lookAhead, false);

        Path path = new Path();
        path.add(new Waypoint(new Vector2d( 0, 0), options));
        path.add(new Waypoint(new Vector2d( 96.0, 0), options));
        path.add(new Waypoint(new Vector2d( 96.0, 72.0), options));
        path.add(new Waypoint(new Vector2d( 0, 72.0), options));
        path.add(new Waypoint(new Vector2d( 0, 0), options));
  
        Path revPath = new Path(path);
        revPath.setReverseOrder();
        revPath.setReverseDirection();
        
        runAction(new PathFollowerAction(path));			// drive forward
        runAction(new PathFollowerAction(revPath));    	// drive reversed 
    }
}
