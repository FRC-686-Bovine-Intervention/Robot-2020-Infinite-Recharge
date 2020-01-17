package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveStraightAction;

/**
 * Just drive in a straight line, using VelocityHeading mode
 */
public class DriveStraightMode extends AutoModeBase {

    public DriveStraightMode() 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Drive Straight");
    	
        runAction(new DriveStraightAction(48.0, 12.0));       		         
    }
}
