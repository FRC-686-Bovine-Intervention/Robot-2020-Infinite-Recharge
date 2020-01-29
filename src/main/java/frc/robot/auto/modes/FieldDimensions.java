package frc.robot.auto.modes;

import frc.robot.Constants;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;

/**
 * Interface that holds all the field measurements 
 */

public class FieldDimensions 
{
    public enum TargetPositionEnum
    {

    }

	// dimensions of field components
	public static double kFieldLengthX = 648;       // 54'
	public static double kFieldLengthY = 324;       // 27'
    





















	private static final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        }
    };
    
    public static DataLogger getLogger() { return logger; }    
}