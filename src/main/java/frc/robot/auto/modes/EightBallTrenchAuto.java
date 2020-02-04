package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.WaitAction;

/**
 * Just drive in a straight line, using VelocityHeading mode
 */
public class EightBallTrenchAuto extends AutoModeBase {

    public EightBallTrenchAuto() 
    { 
    }
    

    @Override
    protected void routine() throws AutoModeEndedException 
    {
        boolean doneShooting = true;

        backupToTrenchPath = new Path();     //lineup using limelight feed
        backupToTrenchPath.add(new Waypoint());//start line
        backupToTrenchPath.add(new Waypoint());//turn position
        backupToTrenchPath.add(new Waypoint());//turn on limelights
        //may need to be reversed 

        //new path drive forward path while intaking and lineup to shoot using limelights
        intakeTrenchPath = new Path();
        intakeTrenchPath.add(new Waypoint());//turn on limelights end point
        intakeTrenchPath.add(new Waypoint()); //a foot away from last ball intaked
        //May need to be reversed

        // Actions--------------------------------------------------------------------------------------------------------------------------------
        //run action - shoot
        runAction(new ShootAction);
        runAction(new WaitAction(doneShooting));
        //backup path to start of trench
        runAction(new PathFollowerAction(backupToTrenchPath));
        //intake action
        runAction(new IntakeAction);
        //drive forward path while intaking
        runAction(new PathFollowerAction(intakeTrenchPath));
        //lineup turret action
        runAction(new TurretAction);
        //run action - shoot   
        runAction(new ShootAction); 		         
    }
}
