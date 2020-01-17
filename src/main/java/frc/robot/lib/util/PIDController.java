package frc.robot.lib.util;


/**
 * This class implements a PID Control Loop.
 * 
 * Does all computation synchronously (i.e. the calculate() function must be
 * called by the user from his own thread)
 */
public class PIDController {
    private double kP; 					// factor for "proportional" control
    private double kI; 					// factor for "integral" control
    private double kD; 					// factor for "derivative" control
    private double maxOut = 1.0; 		// |maximum output|
    private double minOut = -1.0; 		// |minimum output|
    private double maxIn = 0.0; 		// maximum input - limit setpoint to this
    private double minIn = 0.0; 		// minimum input - limit setpoint to this
    private boolean continuous = false; // do the endpoints wrap around? eg. Absolute encoder
    private double prevError = 0.0; 	// the prior sensor input (used to compute velocity)
    private double totalError = 0.0; 	// the sum of the errors for use in the integral calc
    private double setpoint = 0.0;
    private double error = 0.0;
    private double result = 0.0;
    private double lastInput = Double.NaN;
    private double deadband = 0.0; 		// If the absolute error is less than deadband
                                     	// then treat error for the proportional term as 0

    public PIDController() {}

    /**
     * Allocate a PID object with the given constants for P, I, D
     */
    public PIDController(double _kp, double _ki, double _kd) 
    {
        kP = _kp;
        kI = _ki;
        kD = _kd;
    }

    /**
     * Read the input, calculate the output accordingly, and write to the
     * output. This should be called at a constant rate by the user (ex. in a
     * timed thread)
     */
    public double calculate(double input) 
    {
        lastInput = input;
        error = setpoint - input;
        if (continuous) 
        {
            if (Math.abs(error) > (maxIn - minIn) / 2) 
            {
                if (error > 0) 
                    error = error - maxIn + minIn;
                else 
                    error = error + maxIn - minIn;
            }
        }

        if ((error * kP < maxOut) && (error * kP > minOut)) 
            totalError += error;
        else
            totalError = 0;

        // Don't blow away m_error so as to not break derivative
        double proportionalError = Math.abs(error) < deadband ? 0 : error;
        double diffError = error - prevError;
        
        result = (kP * proportionalError + kI * totalError + kD * diffError);
        prevError = error;

        result = Util.limit(result, minOut, maxOut);
        return result;
    }

    /**
     * Set the PID controller gain parameters. Set the proportional, integral,
     * and differential coefficients.
     */
    public void setPID(double _kp, double _ki, double _kd) 
    {
        kP = _kp;
        kI = _ki;
        kD = _kd;
    }

    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }

    /**
     * Return the current PID result This is always centered on zero and
     * constrained the the max and min outs
     */
    public double get() { return result; }

    /**
     * Set the PID controller to consider the input to be continuous, Rather
     * then using the max and min in as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the
     * setpoint.
     */
    public void setContinuous() { this.setContinuous(true); }
    public void setContinuous(boolean _continuous) { continuous = _continuous; }

    public void setDeadband(double _deadband) { deadband = _deadband; }

    /**
     * Sets the maximum and minimum values expected from the input.
     */
    public void setInputRange(double _minimumInput, double _maximumInput) 
    {
        if (_minimumInput > _maximumInput) 
        {
            // throw new BoundaryException("Lower bound is greater than upper bound");
        }
        minIn = _minimumInput;
        maxIn = _maximumInput;
        setSetpoint(setpoint);
    }

    /**
     * Sets the minimum and maximum values to write.
     */
    public void setOutputRange(double _minimumOutput, double _maximumOutput) 
    {
        if (_minimumOutput > _maximumOutput) 
        {
            // throw new BoundaryException("Lower bound is greater than upper bound");
        }
        minOut = _minimumOutput;
        maxOut = _maximumOutput;
    }

    /**
     * Set the setpoint for the PID controller
     */
    public void setSetpoint(double _setpoint) 
    {
        if (maxIn > minIn) 
        {
            if (_setpoint > maxIn) 
                setpoint = maxIn;
            else if (_setpoint < minIn) 
                setpoint = minIn;
            else
                setpoint = _setpoint;
        } 
    	else
    	{
            setpoint = _setpoint;
    	}
    }

    /**
     * Returns the current setpoint of the PID controller
     */
    public double getSetpoint() { return setpoint; }

    public double getError() { return error; }

    /**
     * Return true if the error is within the tolerance
     */
    public boolean onTarget(double tolerance) 
    {
        return ((lastInput != Double.NaN) && (Math.abs(lastInput - setpoint) < tolerance));
    }

    /**
     * Reset all internal terms.
     */
    public void reset() 
    {
        lastInput = Double.NaN;
        prevError = 0;
        totalError = 0;
        result = 0;
        setpoint = 0;
    }

    public void resetIntegrator() 
    {
        totalError = 0;
    }

    public String getState() 
    {
        String state = "";

        state += "Kp: " + kP + "\n";
        state += "Ki: " + kI + "\n";
        state += "Kd: " + kD + "\n";

        return state;
    }

    public String getType() 
    {
        return "PIDController";
    }
}
