package frc.robot.vision;

import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

import frc.robot.lib.util.Vector2d;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Timer;

/**
 * A class that is used to keep track of all goals detected by the vision
 * system. As goals are detected/not detected anymore by the vision system,
 * function calls will be made to create, destroy, or update a goal track.
 * 
 * This helps in the goal ranking process that determines which goal to fire
 * into, and helps to smooth measurements of the goal's location over time.
 * 
 * @see GoalTracker.java
 */
public class GoalTrack
{
	Map<Double, Vector2d> observedPositions = new TreeMap<>();
	Vector2d smoothedPosition = null;
	int trackId;

 
	private GoalTrack()
	{
	}

	/**
	 * Makes a new track based on the timestamp and the goal's coordinates (from
	 * vision)
	 */
	public static GoalTrack makeNewTrack(double timestamp, Vector2d firstObservation, int id)
	{
		GoalTrack track = new GoalTrack();
		track.observedPositions.put(timestamp, firstObservation);
		track.smoothedPosition = firstObservation;
		track.trackId = id;
		return track;
	}

	public void emptyUpdate()
	{
		pruneByTime();
	}

	/**
	 * Attempts to update the track with a new observation.
	 * 
	 * @return True if the track was updated
	 */
	public boolean tryUpdate(double timestamp, Vector2d newObservation)
	{
		if (!isAlive())
		{
			return false;
		}
		double distance = smoothedPosition.distance(newObservation);
		if (distance < GoalTracker.kMaxTrackerDistance)
		{
			observedPositions.put(timestamp, newObservation);
			pruneByTime();
			return true;
		} 
		else
		{
			// new observation was too far from smoothed observation -- ignore it
			emptyUpdate();
			return false;
		}
	}

	public boolean isAlive()
	{
		return observedPositions.size() > 0;
	}

	/**
	 * Removes the track if it is older than the set "age" described in the
	 * Constants file.
	 * 
	 * @see Constants.java
	 */
	void pruneByTime()
	{
		double deleteBefore = Timer.getFPGATimestamp() - GoalTracker.kGoalTrackAveragePeriod;
		for (Iterator<Map.Entry<Double, Vector2d>> it = observedPositions.entrySet().iterator(); it.hasNext();)
		{
			Map.Entry<Double, Vector2d> entry = it.next();
			if (entry.getKey() < deleteBefore)
			{
				it.remove();
			}
		}
		if (observedPositions.isEmpty())
		{
			smoothedPosition = null;
		} 
		else
		{
			smooth();
		}
	}

	/**
	 * Averages out the observed positions based on an set of observed positions
	 */
	void smooth()
	{
		if (isAlive())
		{
			double x = 0;
			double y = 0;
			for (Map.Entry<Double, Vector2d> entry : observedPositions.entrySet())
			{
				x += entry.getValue().getX();
				y += entry.getValue().getY();
			}
			x /= observedPositions.size();
			y /= observedPositions.size();
			smoothedPosition = new Vector2d(x, y);
		}
	}

	public Vector2d getSmoothedPosition()
	{
		return smoothedPosition;
	}

	public double getLatestTimestamp()
	{
		return observedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
	}

	public double getStability()
	{
		return Math.min(1.0, observedPositions.size() / (Constants.kCameraFrameRate * GoalTracker.kGoalTrackAveragePeriod));
	}

	public int getId()
	{
		return trackId;
	}
}
