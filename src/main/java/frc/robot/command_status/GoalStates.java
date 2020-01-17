package frc.robot.command_status;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;


// Sorted list of range & bearing to each tracked target
// First item in list has the highest score (determined by GoalTracker.TrackReportComparator)

public class GoalStates
{
	private static GoalStates instance = new GoalStates();	
	public static GoalStates getInstance()
	{
		return instance;
	}

	static List<GoalState> goalList = new ArrayList<>();	

	synchronized public void clear() 
	{	
		goalList.clear();	
	
	}
	synchronized public void add(Vector2d _fieldToGoal, Pose _fieldToShooter, int _trackId, double _time)
	{
		goalList.add(new GoalState(_fieldToGoal, _fieldToShooter, _trackId, _time));
	}

	synchronized public boolean targetFound() { return !goalList.isEmpty(); }
	
	synchronized public Optional<GoalState> getBestVisionTarget()
	{
		Optional<GoalState> rv;
		
		if (goalList.isEmpty())
		{
			rv = Optional.empty();
		}
		else
		{
			rv = Optional.of(goalList.get(0));	// goalList is sorted so that goalList.get(0) has the best target
		}
		return rv;
	}
	

	
	
	 // A container class to specify the range and angle of a goal
	 // with respect to a robot's position and heading. angle. 
	 // It also contains the computer vision's track's ID.
	
	public class GoalState 
	{
		Vector2d fieldToGoal;
	    double horizontalDistance;	// in inches
	    double relativeBearing;		// in radians, relative to robot's heading
	    int trackId;
	    double time;				// not really needed, just for debug to see that GoalState is still being updated

	    public GoalState(Vector2d _fieldToGoal, Pose _fieldToShooter, int _trackId, double _time)
	    {
			// find relative distance and bearing to goal
			Vector2d shooterToGoal = _fieldToGoal.sub(_fieldToShooter.getPosition());
	    	
			double distanceToGoal = shooterToGoal.length();
			double bearingToGoal = shooterToGoal.angle() - _fieldToShooter.getHeading(); 	// bearing relative to shooter's heading
		
			fieldToGoal = _fieldToGoal;
	        horizontalDistance = distanceToGoal;
	        relativeBearing = bearingToGoal;
	        trackId = _trackId;
	        time = _time;
	    }

	    public Vector2d getPosition() { return fieldToGoal; }
	    public double getHorizontalDistance() { return horizontalDistance; }
	    public double getRelativeBearing() { return relativeBearing; }
	    public int getTrackId() { return trackId; }
		public double getTrackTime() { return time; }
		
		public String toString() 
		{
			return "Position:" + fieldToGoal + ", horizDist:" + horizontalDistance + ", relBearing:" + relativeBearing + ", trackID:" + trackId + ", trackTime:" + time;
		}
	}	





	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
			GoalStates goalStates = GoalStates.getInstance();
			put("GoalState/numTargets", goalList.size());
			
			Optional<GoalState> optTarget = goalStates.getBestVisionTarget();
			if (optTarget.isPresent())
			{
				GoalState target = optTarget.get();
				put("GoalState/bestTargetX", target.getPosition().getX());
				put("GoalState/bestTargetY", target.getPosition().getY());
				put("GoalState/bestTargetRange", target.getHorizontalDistance());
				put("GoalState/bestTargetBearing", target.getRelativeBearing());
				put("GoalState/bestTargetTime", target.getTrackTime());
			}
			else
			{
				put("GoalState/bestTargetX", -999);
				put("GoalState/bestTargetY", -999);
				put("GoalState/bestTargetRange", -999);
				put("GoalState/bestTargetBearing", -999);
				put("GoalState/bestTargetTime", -999);
			}
		}
	};

	public DataLogger getLogger()
	{
		return logger;
	}
}
