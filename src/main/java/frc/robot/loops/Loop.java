package frc.robot.loops;

/**
 * Attribution: adapted from FRC Team 254
 */

/**
 * Interface for loops, which are routines that run periodically.
 * Loops are typically registered to LoopControllers.
 * LoopControllers will call the onStart, onLoop, and onStop 
 * functions of each loop at the appropriate times. 
 */
public interface Loop 
{
    public void onStart();

    public void onLoop();

    public void onStop();
}
