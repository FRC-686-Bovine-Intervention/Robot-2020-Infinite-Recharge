package frc.robot.lib.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * PulseTrain
 * <p>
 * Creates a finite pulse train with user-defined on-time, off-time, pulse count, polarity.
 */
public class PulseTrain
{
    int endCount = 0;
    int pulseCount = 0;
    double onTime = 0.0;
    double offTime = 0.0;

    boolean enabled = false;        // true when pulse train is active
    boolean state = false;
    boolean userPolarity = false;   // output will start and end in userPolarity
    boolean finished = false;       // true for a single cycle when complete
    double startTime;

    public PulseTrain(int _numPulses, double _onTime, double _offTime) 
    {
        this(_numPulses, _onTime, _offTime, false);
    }

    public PulseTrain(int _numPulses, double _onTime, double _offTime, boolean _userPolarity) 
    {
        endCount = _numPulses;
        onTime = _onTime;
        offTime = _offTime;
        userPolarity = _userPolarity;
        pulseCount = 0;
        state = false;
        enabled = false;
    }

    public void start()
    {
        enabled = true;
        startTime = Timer.getFPGATimestamp();
        state = true;
        pulseCount = 0;
    }

    public boolean update()
    {
        finished = false;

        if (enabled)
        {
            double currentTime = Timer.getFPGATimestamp();
            double elapsedTime = currentTime - startTime;

            if (state)
            {
                if (elapsedTime >= onTime)
                {
                    startTime = currentTime;
                    state = false;

                    // count number of pulses, stop when end count is reached
                    pulseCount++;
                    if (pulseCount >= endCount)
                    {
                        enabled = false;
                        finished = true;
                    }
                }
            }
            else
            {
                if (elapsedTime >= offTime)
                {
                    startTime = currentTime;
                    state = true;
                }
            }
        }

        return state;
    }
    public boolean getEnabled()
    {
        return enabled;
    }
    public boolean getFinished()
    {
        return finished;
    }
}