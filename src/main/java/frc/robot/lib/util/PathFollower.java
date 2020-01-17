package frc.robot.lib.util;

import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.GoalStates;
import frc.robot.command_status.RobotState;
import frc.robot.command_status.GoalStates.GoalState;
import frc.robot.lib.util.Kinematics.WheelSpeed;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.GoalStateLoop;
import frc.robot.subsystems.Drive;

import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;


/**
 * Action for following a path defined by a Path object.
 * 
 * Serially configures a PathFollower object to follow each path 
 */
public class PathFollower 
{ 
	/* Controller can be started in PATH_FOLLOWING or VISION state, as set by _initialState in constructor
	 * If in PATH_FOLLOWING state, it will transition to VISION state when the robot locates a vision target.  
	 * It will only search for vision targets on vision-enabled path segments (set in PathSegmentOptions)
	 * It will not transition back to PATH_FOLLOWING.
	 * Start a new Action to continue following a path moving after this Action has completed.
	 */
	public enum PathVisionState { PATH_FOLLOWING, VISION };
	
	public PathVisionState state;
	
	public Drive drive = Drive.getInstance();
	public RobotState robotState = RobotState.getInstance();
	public GoalStates goalStates = GoalStates.getInstance();
	
	Path path;
	
	public double distanceFromPath;
	public double lookaheadDist;
	public Vector2d lookaheadPoint = new Vector2d();
	public double bearingToTarget;		
	
	public double currentTime;

	public Pose currentPose = new Pose();
	public Pose previousPose = new Pose();
	
	public double prevDistanceToTargetInches;
	public double prevHeadingToTarget;
	
	public double distanceToTargetInches;

	public double remainingDistance;

	public double speed;
	public double curvature;

	public WheelSpeed wheelSpeed;
	
	private double prevSpeed;
	private double prevTime;

	public boolean haveGoal;
	public Optional<Vector2d> currentFieldToGoal = Optional.empty();
    double kTargetDistanceThresholdFromCenterInches;

	
    public PathFollower(Path _path, PathVisionState _initialState) 
    {
        drive = Drive.getInstance();
        path = _path;
        state = _initialState;
    }

    public void start() 
    {
		prevSpeed = robotState.getSpeed();
		prevTime  = -1;		
        remainingDistance = Double.MAX_VALUE;	// make sure we run update() at least once before finishing
    }


    public void update() 
    {
		//---------------------------------------------------
		// Get inputs
		//---------------------------------------------------
		
		currentPose = robotState.getLatestFieldToVehicle();		
		currentTime = Timer.getFPGATimestamp();

		//---------------------------------------------------
		// Process
		//---------------------------------------------------
		wheelSpeed = pathVisionDrive(currentTime, currentPose);		// sets speed, curvature to follow path

		//---------------------------------------------------
		// Output: Send drive control
		//---------------------------------------------------
        drive.setVelocitySetpoint(wheelSpeed);
	}

    
	public WheelSpeed pathVisionDrive(double _currentTime, Pose _currentPose)
	{
		if (prevTime < 0)				// initial setting of prevTime is important to limit initial acceleration
			prevTime = _currentTime;	// avoid calling Timer.getFPGATimestamp() in this function to allow off-robot testing
		
		System.out.println("At " + _currentPose + "  Driving to " + path.getSegmentEnd());
		
		remainingDistance = Double.MAX_VALUE;
		double finalSpeed = 0;
		double maxSpeed = 0;
		double maxAccel = 0;
		
		boolean visionEnabledSegment = path.getSegmentVisionEnable(); 
		if (visionEnabledSegment)
		{
			visionDrive(_currentTime, _currentPose);
		}
		else
		{
			// GoalStateLoop.getInstance().resetVision();
			pathDrive(_currentTime, _currentPose);
		}
			
		if (state == PathVisionState.PATH_FOLLOWING)	 
		{
			remainingDistance = path.getRemainingLength();		// TODO: address stopping when past final segment
			finalSpeed = path.getSegmentFinalSpeed();
			maxSpeed = path.getSegmentMaxSpeed();
			maxAccel = path.getSegmentMaxAccel();
		}
		else
		{
			remainingDistance = Math.max(distanceToTargetInches, 0.0);   // keep maxSpeed = 0 when we pass the target			
			finalSpeed = path.getSegmentFinalSpeed();
			maxSpeed = Constants.kVisionMaxVel;
			maxAccel = Constants.kVisionMaxAccel;
		}
		
		speedControl(_currentTime, remainingDistance, finalSpeed, maxSpeed, maxAccel);

		if (path.getReverseDirection())
		{
			speed = -speed;
			curvature = -curvature;	// TODO: simplify by removing this, and removing flipping heading 180 degrees below?
		}
		
		wheelSpeed = Kinematics.inverseKinematicsFromSpeedCurvature(speed, curvature);
		wheelSpeed.limit(maxSpeed);
		return wheelSpeed;
	}
	
	public WheelSpeed getWheelVelocity() { return getWheelVelocity(); }
	public Path   getPath() { return path; }	// warning: not returning a defensive copy
	public double getDistanceFromPath() { return distanceFromPath; }
	public PathVisionState getPathVisionState() { return state; }

	
	// Drive towards lookahead point on path
	private void pathDrive(double _currentTime, Pose _currentPose)
	{
		state = PathVisionState.PATH_FOLLOWING;

		//---------------------------------------------------
		// Find Lookahead Point
		//---------------------------------------------------
		distanceFromPath = path.update(_currentPose.getPosition());
		lookaheadPoint = path.getLookaheadPoint(_currentPose.getPosition(), distanceFromPath);
		
		//---------------------------------------------------
		// Find arc to travel to Lookahead Point
		//---------------------------------------------------
		Vector2d robotToTarget = lookaheadPoint.sub(_currentPose.getPosition());
		lookaheadDist = robotToTarget.length();
		bearingToTarget = robotToTarget.angle() - _currentPose.getHeading();
		if (path.getReverseDirection())
			bearingToTarget -= Math.PI;	// flip robot around
		
		curvature = 2 * Math.sin(bearingToTarget) / lookaheadDist;
	}

	Vector2d fieldToGoal = new Vector2d();
	Pose fieldToShooter = new Pose();
	double distanceToGoal = -999;
	double bearingToGoal = -999;

	
	// drive towards vision target (or follow path if no target acquired)
	public void visionDrive(double _currentTime, Pose _currentPose)
	{
		// update currentGoalState based on whether target is currently seen, and if button is being pressed
		Optional<GoalState> visionGoalState = goalStates.getBestVisionTarget();
		if (visionGoalState.isPresent())
		{
			currentFieldToGoal = Optional.of( visionGoalState.get().getPosition() );
		}
		else
		{
			boolean enabled = true;	// replaces checking for driver assistance button
			if (enabled)
			{
				// target not currently seen, but button is still pressed
				// --> keep same currentFieldToGoal
			}
			else
			{
				// target not currently seen, button not pressed
				currentFieldToGoal = Optional.empty();
			}
		}

		haveGoal = currentFieldToGoal.isPresent();

		// If we get a valid message from the Vision co-processor, update our estimate of the target location
		if (haveGoal)
		{
			state = PathVisionState.VISION;

            // Get range and angle to target
            fieldToGoal = currentFieldToGoal.get();
            fieldToShooter = RobotState.getInstance().getFieldToShooter(currentTime);
		    Vector2d shooterToGoal = fieldToGoal.sub(fieldToShooter.getPosition());
	    	distanceToGoal = shooterToGoal.length();
			bearingToGoal = shooterToGoal.angle() - fieldToShooter.getHeading(); 	// bearing relative to shooter's heading

			kTargetDistanceThresholdFromCenterInches = Constants.kHatchTargetDistanceThresholdFromCenterInches;			
            distanceToTargetInches = distanceToGoal - kTargetDistanceThresholdFromCenterInches;   // distance from camera
            bearingToTarget = bearingToGoal;

			// Calculate motor settings to turn towards target
			lookaheadDist = Math.min(Constants.kVisionLookaheadDist, distanceToTargetInches);	// length of chord <= kVisionLookaheadDist
			curvature     = 2 * Math.sin(bearingToTarget) / lookaheadDist;						// curvature = 1/radius of circle (positive: turn left, negative: turn right)
		}
		else
		{
			// target not acquired -- speed/curvature will be controlled by path follower
			pathDrive(_currentTime, _currentPose);
		}
	}

	
	// keep speed within acceleration limits
	public void speedControl(double _currentTime, double _remainingDistance, double _finalSpeed, double _maxSpeed, double _maxAccel)
	{
		//---------------------------------------------------
		// Apply speed control
		// Note: speed will always be positive in this function
		// it will be made negative before applying to motors
		// if path is reversed
		//---------------------------------------------------
		speed = _maxSpeed;
		
		double dt = _currentTime - prevTime;
		
		// apply acceleration limits
		double accel = (speed - prevSpeed) / dt;
		if (accel > _maxAccel)
			speed = prevSpeed + _maxAccel * dt;
		else if (accel < -_maxAccel)
			speed = prevSpeed - _maxAccel * dt;

		// apply braking distance limits
		// vf^2 = v^2 + 2*a*d   Solve for v, configured vf, a, and measured d
		double stoppingDistance = _remainingDistance;
		double maxBrakingSpeed = Math.sqrt(_finalSpeed * _finalSpeed + 2.0 * _maxAccel * stoppingDistance);
		if (Math.abs(speed) > maxBrakingSpeed)
			speed = Math.signum(speed) * maxBrakingSpeed;

		// apply minimum velocity limit (Talons can't track low speeds well)
		final double kMinSpeed = 4.0;
		if (Math.abs(speed) < kMinSpeed) 
			speed = Math.signum(accel) * kMinSpeed;

		// store for next time through loop	
		prevTime = _currentTime;
		prevSpeed = speed;
	}
		
	
    public boolean isFinished() 
    {
    	boolean done = false;
    	
    	if (state == PathVisionState.PATH_FOLLOWING)
	        done = (remainingDistance <= DriveLoop.kPathFollowingCompletionTolerance);
    	else
    		done = (remainingDistance <= Constants.kVisionCompletionTolerance);
    	
     	return done;
    }

    public void done() 
    {
		// cleanup code, if any
    	drive.setVelocitySetpoint(new WheelSpeed(path.finalSpeed, path.finalSpeed));
    }

 
    
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {            
			DriveCommand cmd = drive.getCommand();
    		put("PathVision/driveMode", cmd.getDriveControlMode().toString() );
    		put("PathVision/talonMode", cmd.getTalonControlMode().toString() );
    		put("PathVision/left",  cmd.getLeftMotor() );
       		put("PathVision/right", cmd.getRightMotor() );
      		put("PathVision/neutralMode", DriveCommand.getNeutralMode().toString() );
      		
       		Pose odometry = robotState.getLatestFieldToVehicle();
            put("PathVision/positionX",  odometry.getX());
            put("PathVision/positionY",  odometry.getY());
            put("PathVision/headingDeg", odometry.getHeadingDeg());
        	
			put("PathVision/reversed", path.getReverseDirection());
			put("PathVision/state", state.toString());

			put("PathVision/segmentStartX", path.getSegmentStart().getX());
			put("PathVision/segmentStartY", path.getSegmentStart().getY());
			put("PathVision/segmentEndX", path.getSegmentEnd().getX());
			put("PathVision/segmentEndY", path.getSegmentEnd().getY());
			put("PathVision/segmentFinalSpeed", path.getSegmentFinalSpeed());
			put("PathVision/segmentMaxSpeed", path.getSegmentMaxSpeed());
			put("PathVision/segmentVisionEnable", path.getSegmentVisionEnable());
			
			put("PathVision/distanceFromPath", distanceFromPath );
			put("PathVision/lookaheadDist", lookaheadDist );
			put("PathVision/lookaheadPointX",  lookaheadPoint.getX() );
			put("PathVision/lookaheadPointY",  lookaheadPoint.getY());

			put("PathVision/prevPoseX", previousPose.getX());
			put("PathVision/prevPoseY", previousPose.getY());
			
			put("PathVision/remainingDistance",  remainingDistance );
			
			put("PathVision/speed", 	speed);
			put("PathVision/curvature", curvature );
			put("PathVision/lSpeed", 	wheelSpeed.left);
			put("PathVision/rSpeed", 	wheelSpeed.right);

			put("PathVision/visionEnabledSegment", path.getSegmentVisionEnable());
			put("PathVision/state", state.toString());
 			
			if (currentFieldToGoal.isPresent())
			{
				put("PathVision/fieldToGoalX", fieldToGoal.getX());
				put("PathVision/fieldToGoalY", fieldToGoal.getY());
				put("PathVision/fieldToShooterX", fieldToShooter.getX());
				put("PathVision/fieldToShooterY", fieldToShooter.getY());
				put("PathVision/distanceToGoal", distanceToGoal);
				put("PathVision/bearingToGoal", bearingToGoal);
			}
			else{
				put("PathVision/fieldToGoalX", -999);
				put("PathVision/fieldToGoalY", -999);
				put("PathVision/fieldToShooterX", -999);
				put("PathVision/fieldToShooterY", -999);
				put("PathVision/distanceToGoal", -999);
				put("PathVision/bearingToGoal", -999);
			}

       }
    };
	
    public DataLogger getLogger() { return logger; }
}
