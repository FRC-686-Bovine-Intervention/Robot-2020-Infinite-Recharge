package frc.robot.loops;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.command_status.GoalStates;
import frc.robot.command_status.GoalStates.GoalState;
import frc.robot.command_status.RobotState;
import frc.robot.lib.joystick.SelectedDriverControlsReversible;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.vision.GoalTracker;
import frc.robot.vision.VisionTargetList;

/**
 * This function adds vision updates (from the Nexus smartphone) to a list in
 * RobotState. This helps keep track of goals detected by the vision system. The
 * code to determine the best goal to shoot at and prune old Goal tracks is in
 * GoalTracker.java
 * 
 * @see GoalTracker.java
 */
public class GoalStateLoop implements Loop
{
	static GoalStateLoop instance = new GoalStateLoop();
	boolean visionUpdatePending = false; // set to true when updated vision information is available 
										 // (set from VisionServer thread)

	RobotState robotState = RobotState.getInstance();
	VisionTargetList visionTargetList = VisionTargetList.getInstance();

	public GoalTracker goalTracker = new GoalTracker();
	GoalStates goalStates = GoalStates.getInstance();

	int currentBestTrackId = -1;
	
	enum RangeMethod { DIFFERENTIAL_HEIGHT, TARGET_HEIGHT, TARGET_WIDTH };
	RangeMethod rangeMethod = RangeMethod.TARGET_HEIGHT; 
	// RangeMethod rangeMethod = RangeMethod.TARGET_WIDTH; 
	
	public static GoalStateLoop getInstance()
	{
		return instance;
	}

	GoalStateLoop()
	{
	}

	@Override
	public void onStart() {}

	@Override
	public void onLoop()
	{
		updateGoalLocations();
	}

	@Override
	public void onStop()
	{
		// no-op
	}

	double hAngle, vAngle, hWidth, vWidth, range, horizontalDistance;

	private void updateGoalLocations()
	{
		// Step 1: Find location of goals in this image with respect to field
		
		double imageCaptureTimestamp = visionTargetList.getImageCaptureTimestamp();
		List<VisionTargetList.Target> visionTargets = visionTargetList.getTargets();
		Pose fieldToCamera = robotState.getFieldToCamera(imageCaptureTimestamp);	// find position of camera back when image was taken (removes latency in processing)

		double kCameraPoseThetaRad = 0.0;	// no variation from PI in Constants
		double kCameraPitchRad = Constants.kHatchCameraPitchRad;
		double kCameraPoseZ = Constants.kHatchCameraPoseZ;
		
		if (SelectedDriverControlsReversible.getInstance().getDrivingCargo())
		{
			kCameraPoseThetaRad = 0.0;
			kCameraPitchRad = Constants.kCargoCameraPitchRad;
			kCameraPoseZ = Constants.kCargoCameraPoseZ;
		}


		List<Vector2d> fieldToGoals = new ArrayList<>();
		
		if (!(visionTargets == null || visionTargets.isEmpty()))
		{
			for (VisionTargetList.Target target : visionTargets)
			{
				hAngle = target.getHorizontalAngle() - kCameraPoseThetaRad;	// compensate for camera yaw
				vAngle = target.getVerticalAngle()   - kCameraPitchRad;		// compensate for camera pitch
				hWidth = target.getHorizontalWidth();
				vWidth = target.getVerticalWidth();
						
		        double differentialHeight = Constants.kCenterOfTargetHeightInches - kCameraPoseZ;	
				horizontalDistance = 0;
				range = 0;
				
				switch (rangeMethod)
				{
				case DIFFERENTIAL_HEIGHT:
					horizontalDistance = Math.abs(differentialHeight / Math.tan(vAngle));
					break;
					
				case TARGET_WIDTH:
					// assumes target is horizontally perpendicular to camera axis (not likely unless you attempt to make it so)
					range = (Constants.kTargetWidthInches/2.0) / Math.tan(hWidth/2.0);
					horizontalDistance = Math.abs(range * Math.cos(hAngle));
					break;
					
				case TARGET_HEIGHT:
				default:
					// assumes target is vertical
					// using Law of Sines
					range = (Constants.kTargetHeightInches/2.0) * Math.sin(Math.PI/2.0-vAngle-vWidth/2.0) / Math.sin(vWidth/2.0);
					horizontalDistance = Math.abs(range * Math.cos(vAngle));
					break;
				}
				
				if (horizontalDistance > 0)
				{
					Pose cameraToTarget = new Pose( Vector2d.magnitudeAngle(horizontalDistance, hAngle) );
					Pose fieldToTarget = cameraToTarget.changeCoordinateSystem( fieldToCamera );
					fieldToGoals.add( fieldToTarget.getPosition() );
				}
			}
		}
		else
		{
			boolean break_here = true; 	
		}
		
	
		
		
		// Step 2: Add these goals to goal tracker
		goalTracker.update(imageCaptureTimestamp, fieldToGoals);
		
		

		
		// Step 3: 	Rank each goal, sort goals by rank
		//			Store position of goals, calculate range/bearing from shooter to each goal
        double now = Timer.getFPGATimestamp();
        Optional<GoalState> currentTarget = goalStates.getBestVisionTarget();
		Pose predictedFieldToShooter = robotState.getPredictedFieldToShooter(Constants.kAutoAimPredictionTime);

		goalStates.clear();
		for (GoalTracker.TrackReport report : goalTracker.getSortedTrackReports(now, currentTarget))
		{
			goalStates.add(report.fieldToGoal, predictedFieldToShooter, report.trackId, report.getLatestTimestamp());
		}		

	}
	
	
	public synchronized void resetVision()
	{
		goalTracker.reset();
	}


	public GoalTracker getGoalTracker() { return goalTracker; }



	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			put("GoalStateLoop/rangeMethod", rangeMethod.toString());
			put("GoalStateLoop/hAngle", hAngle);
			put("GoalStateLoop/vAngle", vAngle);
			put("GoalStateLoop/hWidth", hWidth);
			put("GoalStateLoop/vWidth", vWidth);
			put("GoalStateLoop/range", range);
			put("GoalStateLoop/horizontalDistance", horizontalDistance);
        }
    };
    
    public DataLogger getLogger() { return logger; }

}
