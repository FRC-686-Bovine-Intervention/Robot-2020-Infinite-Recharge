package frc.robot.lib.util;

import frc.robot.loops.DriveLoop;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the
 * wheelbase as a differential drive (with a corrective factor to account for
 * the inherent skidding of the center 4 wheels quasi-kinematically).
 */

public class Kinematics 
{
	
    public static class LinearAngularSpeed
    {
        public final double linearSpeed;		// change in position in inches
        public final double angularSpeed;		// change in heading in radians

        public LinearAngularSpeed(double _linearSpeed, double _angularSpeed)
        {
            linearSpeed  = _linearSpeed;
            angularSpeed = _angularSpeed;
        }
    }
    
    
    
    public static class WheelSpeed 
    {
        public double left;
        public double right;

        public WheelSpeed()
		{
			this(0,0);
		}
        
        public WheelSpeed(WheelSpeed _speed) 
        {
            this(_speed.left, _speed.right);
        }
        
        public WheelSpeed(double _left, double _right) 
        {
            left = _left;
            right = _right;
        }
        
		public void scale(double _scale)
        {
            left  *= _scale;
            right *= _scale;
        }
        
    	public void limit(double _limit)
    	{
            // Scale the command to respect the max wheel velocity limits
            double maxSpeed = Math.max(Math.abs(left), Math.abs(right));
            if (maxSpeed > _limit)
            	scale(_limit/maxSpeed);
    	}
    }

    
    
    /**
     * Forward kinematics using only encoders, rotation is implicit (less
     * accurate than below, but useful for predicting motion)
     */
	// return linear and angular velocity that result from left/right wheel speeds
	// can also get linear distance and change in heading from change in left/right wheel distance over a period of time
	//        (replace left/right speed with change in left/right distance)
    public static LinearAngularSpeed forwardKinematics(double _lSpeed, double _rSpeed)
    {
    	double linearSpeed = (_lSpeed + _rSpeed)/2;		// linear speed of center of robot is the average of the left and right
    	double dSpeed  = (_rSpeed - _lSpeed)/2;			// differential speed of wheels (positive: turning to left, increasing theta)
    	double angularSpeed = dSpeed * 2 * DriveLoop.kTrackScrubFactor / DriveLoop.kTrackEffectiveDiameter;		// angular velocity (in rad/sec) due to differential speed
        return new LinearAngularSpeed(linearSpeed, angularSpeed);			
    }
    
    /**
     * Forward kinematics using encoders and explicitly measured rotation (ie. from gyro)
     */

    public static LinearAngularSpeed forwardKinematics(double _lSpeed, double _rSpeed, double _angularSpeed)
    {
        return new LinearAngularSpeed((_lSpeed + _rSpeed)/2, _angularSpeed);
    }

    
    /** Append the result of forward kinematics to a previous pose. */

    public static Pose integrateForwardKinematics(Pose _currentPose, double _lSpeed, double _rSpeed, double _gyroAngle)
    {
    	LinearAngularSpeed speed = forwardKinematics(_lSpeed, _rSpeed, _gyroAngle - _currentPose.getHeading());
        return travelArc(_currentPose, speed);
    }
    
    // Obtain a new Pose from travel along a constant curvature path.
    public static Pose travelArc(Pose _initialPose, LinearAngularSpeed _speed)
    {
		double D = _speed.linearSpeed;				// distance traveled = arc-length of circle
		double L = D;							// chord-length
		
		double dTheta = _speed.angularSpeed;
		if (Math.abs(dTheta) > 1e-9)
			L = 2*D*Math.sin(dTheta/2)/dTheta;			// chord-length given change in heading
				
		double avgHeading = _initialPose.getHeading() + dTheta/2;	// mean of current and final headings

		Vector2d translation = Vector2d.magnitudeAngle(L, avgHeading);	// calculate change in position
		
		// update pose
		Pose finalPose = _initialPose.add(translation).turn(_speed.angularSpeed);
		
		return finalPose;
    }

    
    
    
    
    /*
     * Calculate left/right wheel speeds that will give desired robot speed and change of heading
     * 
     * For differentially steered robot with angular velocity w and a wheelbase of L
     * traveling an arc with radius R, set the left and right wheel velocities to:
     * 
     * Vl = w*(R-L/2) = w*R - w*L/2 = V - w*L/2 = V - dV
     * Vr = w*(R+L/2) = w*R + w*L/2 = V + w*L/2 = V + dV
     * 
     * dV = w*L/2 = V/R*L/2 = V*K*L/2
     * 
     * where V = w*R is the linear velocity of the robot
     * and w = V/R = V*K, is the angular velocity of the robot
     * and K=1/R is the curvature of an arc with radius R
     */
    public static WheelSpeed inverseKinematics(LinearAngularSpeed _speed)
    {
    	return inverseKinematics(_speed.linearSpeed, _speed.angularSpeed);
    }

    public static WheelSpeed inverseKinematicsFromSpeedCurvature(double _linearSpeed, double _curvature)
    {
    	double angularSpeed = _linearSpeed * _curvature;
    	return inverseKinematics(_linearSpeed, angularSpeed);
    }
    
    public static WheelSpeed inverseKinematics(double _linearSpeed, double _angularSpeed) 
    {
        double dSpeed = _angularSpeed * DriveLoop.kTrackEffectiveDiameter / (2 * DriveLoop.kTrackScrubFactor);
        return new WheelSpeed(_linearSpeed - dSpeed, _linearSpeed + dSpeed); 
    }
}
