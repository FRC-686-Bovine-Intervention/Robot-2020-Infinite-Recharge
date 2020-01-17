package frc.robot.lib.joystick;

import frc.robot.lib.util.Util;
import frc.robot.command_status.DriveCommand;

/**
 * Library of steering functions
 */
public class SteeringLib
{
    public static enum NonLinearityEnum
    {
        NONE,
        SQUARED,    // increases fine control while still permitting full power
        CUBED;      // increases fine control even more than SQUARED
    }


    // a class to hold the deadbands and non-linearities for both throttle and turn
    public static class DeadbandNonLinearity
    {
        public double throttleDeadband;
        public double turnDeadband;
    
        public NonLinearityEnum throttleNonLinearity;
        public NonLinearityEnum turnNonLinearity;
    
        public DeadbandNonLinearity()
		{
            this(0.0, 0.0, NonLinearityEnum.NONE, NonLinearityEnum.NONE);
        }
        
        public DeadbandNonLinearity(double _throttleDeadband, double _turnDeadband, NonLinearityEnum _throttleNonLinarity, NonLinearityEnum _turnNonLinarity)
		{
            throttleDeadband = _throttleDeadband;
            turnDeadband = _turnDeadband;
            throttleNonLinearity = _throttleNonLinarity;
            turnNonLinearity = _turnNonLinarity;
		}
    }


    // a class to allow functions to return both throttle and turn values
    public static class ThrottleTurn
    {
        public double throttle;
        public double turn;
    
        public ThrottleTurn()
		{
            this(0.0, 0.0);
        }
        
        public ThrottleTurn(double _throttle, double _turn)
		{
            throttle = _throttle;
            turn = _turn;
		}
    }    

    
    // Tank Drive:  directly apply joystick inputs to drive motors
    public static DriveCommand tankDrive(double _lSpeed, double _rSpeed, DeadbandNonLinearity _deadbandNonLinearity) 
    {
       // apply deadband
       double lSpeed = applyDeadband(_lSpeed, _deadbandNonLinearity.throttleDeadband);
       double rSpeed = applyDeadband(_rSpeed, _deadbandNonLinearity.throttleDeadband);

       // apply non-linearity
       lSpeed = applyNonLinearity(lSpeed, _deadbandNonLinearity.throttleNonLinearity);
       rSpeed = applyNonLinearity(rSpeed, _deadbandNonLinearity.throttleNonLinearity);
       
       DriveCommand signal = new DriveCommand(lSpeed, rSpeed);

       return signal;
    }




    // Arcade Drive:  directly apply joystick inputs to drive motors
    public static DriveCommand arcadeDrive(double _throttle, double _turn, DeadbandNonLinearity _deadbandNonLinearity) 
    {
        // apply deadband
        double moveValue   = applyDeadband(_throttle, _deadbandNonLinearity.throttleDeadband);
        double rotateValue = applyDeadband(_turn, _deadbandNonLinearity.turnDeadband);

        // apply non-linearity
        moveValue   = applyNonLinearity(moveValue, _deadbandNonLinearity.throttleNonLinearity);
        rotateValue = applyNonLinearity(rotateValue, _deadbandNonLinearity.turnNonLinearity);
        
        // arcade drive mapping of joysticks to motors
		double lMotorSpeed, rMotorSpeed;
		if (moveValue > 0.0) {
			if (rotateValue > 0.0) {
				lMotorSpeed = moveValue - rotateValue;
				rMotorSpeed = Math.max(moveValue, rotateValue);
			} else {
				lMotorSpeed = Math.max(moveValue, -rotateValue);
				rMotorSpeed = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				lMotorSpeed = -Math.max(-moveValue, rotateValue);
				rMotorSpeed = moveValue + rotateValue;
			} else {
				lMotorSpeed = moveValue - rotateValue;
				rMotorSpeed = -Math.max(-moveValue, -rotateValue);
			}
		}

		DriveCommand signal = new DriveCommand(lMotorSpeed, rMotorSpeed);

		return signal;
	}



    
    // Trigger Drive:  Triggers control forward/reverse throttle.  Left stick controls steering
    public static DriveCommand triggerDrive(double _throttle, double _turn, DeadbandNonLinearity _deadbandNonLinearity, boolean _quickTurn) 
    {
        // apply deadband
        double moveValue   = applyDeadband(_throttle, _deadbandNonLinearity.throttleDeadband);
        double rotateValue = applyDeadband(_turn, _deadbandNonLinearity.turnDeadband);

        // apply non-linearity
        moveValue   = applyNonLinearity(moveValue, _deadbandNonLinearity.throttleNonLinearity);
        rotateValue = applyNonLinearity(rotateValue, _deadbandNonLinearity.turnNonLinearity);
        
	    double lMotorSpeed, rMotorSpeed;
	    if (rotateValue > 0.0) {
	      lMotorSpeed = moveValue;
	      rMotorSpeed = moveValue - 2*rotateValue*moveValue;
	    } else {
	      lMotorSpeed = moveValue + 2*rotateValue*moveValue;
	      rMotorSpeed = moveValue;
	    }
	    
	    // override normal trigger turning when quick turn button is pressed
	    if (_quickTurn)
	    {
		    lMotorSpeed = +rotateValue;
		    rMotorSpeed = -rotateValue;
	    }
	    
	    DriveCommand signal = new DriveCommand(lMotorSpeed, rMotorSpeed);
	   	    
	    return signal;
    }


    public static double mQuickStopAccumulator = 0.0;

    // Trigger Drive:  Triggers control forward/reverse throttle.  Left stick controls steering
    public static DriveCommand cheesyDrive(double _throttle, double _turn, DeadbandNonLinearity _deadbandNonLinearity, 
                                            boolean _quickTurn, double _kTurnSensitivity) 
    {
        // apply deadband
        double throttle = applyDeadband(_throttle, _deadbandNonLinearity.throttleDeadband);
        double turn     = applyDeadband(_turn, _deadbandNonLinearity.turnDeadband);

        // apply non-linearity
        throttle = applyNonLinearity(throttle, _deadbandNonLinearity.throttleNonLinearity);
        turn     = applyNonLinearity(turn, _deadbandNonLinearity.turnNonLinearity);
        
        double overPower;
        double angularPower;

        if (_quickTurn) 
        {
            if (Math.abs(throttle) < 0.2) 
            {
                double alpha = 0.1;
                mQuickStopAccumulator = (1 - alpha) * mQuickStopAccumulator + alpha * Util.limit(turn, 1.0) * 2;
            }
            overPower = 1.0;
            angularPower = turn;
        }
        else
        {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * turn * _kTurnSensitivity - mQuickStopAccumulator;
            if (mQuickStopAccumulator > 1)
                mQuickStopAccumulator -= 1;
            else if (mQuickStopAccumulator < -1)
                mQuickStopAccumulator += 1;
            else
                mQuickStopAccumulator = 0.0;
        }

        double rMotorSpeed = throttle - angularPower;
        double lMotorSpeed = throttle + angularPower;
        
        // scale motor power to keep within limits
        if (lMotorSpeed > 1.0) {
            rMotorSpeed -= overPower * (lMotorSpeed - 1.0);
            lMotorSpeed = 1.0;
        } else if (rMotorSpeed > 1.0) {
            lMotorSpeed -= overPower * (rMotorSpeed - 1.0);
            rMotorSpeed = 1.0;
        } else if (lMotorSpeed < -1.0) {
            rMotorSpeed += overPower * (-1.0 - lMotorSpeed);
            lMotorSpeed = -1.0;
        } else if (rMotorSpeed < -1.0) {
            lMotorSpeed += overPower * (-1.0 - rMotorSpeed);
            rMotorSpeed = -1.0;
        }
        
	    DriveCommand signal = new DriveCommand(lMotorSpeed, rMotorSpeed);
	   	    
	    return signal;
    }   




    static double applyDeadband(double _in, double _deadband)
    {
        double sign = Math.signum(_in);							// get sign
        double mag = Math.abs(_in);								// get magnitude
        double out;
        if (mag < _deadband) {
            out = 0.0;                                          // zero out controls less than deadband
        } else {
            out = 1.0/(1.0-_deadband) * mag - _deadband;		// inputs from _deadband to 1.0 are scaled to outputs from 0 to 1
        }
        out = sign * out;										// reapply the sign
        out = Util.limit(out, 1.0);								// limit to maximum of +/-1
        return out;
    }




    static double applyNonLinearity(double _in, NonLinearityEnum _nonLinearity)
    {
        double out = 0.0;

        switch (_nonLinearity)
        {
            case NONE:
                out = _in;
                break;

            case SQUARED:
                // square the inputs (while preserving the sign) to increase fine control
                // while permitting full power
                if (_in >= 0.0) {
                    out = (_in * _in);
                } else {
                    out = -(_in * _in);
                }
                break;

            case CUBED:
                // cube the inputs to increase fine control while permitting full power
                out = _in * _in * _in;
                break;

        }
        return out;
    }

}
