package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.FeedBallsAction;
import frc.robot.auto.actions.IntakeAction;
import frc.robot.auto.actions.IntakeStopAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.SpeedUpShooterAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.auto.actions.setTurretAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;

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
        
        int doneShooting = 3;   //three is a testing time to make sure we know that a path is about to start
        final double targetDistance;
        final int shooterStop = 0;
        final int ballCount = 3;
        final int ballNumber = 3;


        Path backupToTrenchPath = new Path();     //lineup using limelight feed
        backupToTrenchPath.add(new Waypoint());//start line
        backupToTrenchPath.add(new Waypoint());//turn position
        backupToTrenchPath.add(new Waypoint());//turn on limelights
        //may need to be reversed 

        //new path drive forward path while intaking and lineup to shoot using limelights
        Path intakeTrenchPath = new Path();
        intakeTrenchPath.add(new Waypoint());//turn on limelights end point
        intakeTrenchPath.add(new Waypoint()); //a foot away from last ball intaked
        //May need to be reversed

        // Actions--------------------------------------------------------------------------------------------------------------------------------
        //run action - shoot
        runAction(new setTurretAction());
        runAction(new SpeedUpShooterAction(targetDistance));
        runAction(new FeedBallsAction(ballCount));
        runAction(new WaitAction(doneShooting));
        runAction(new FeedBallsAction(ballNumber));
        runAction(new SpeedUpShooterAction(shooterStop));
        //backup path to start of trench
        runAction(new PathFollowerAction(backupToTrenchPath));
        //intake action
        runAction(new IntakeAction());
        runAction(new IntakeStopAction());
        //drive forward path while intaking
        runAction(new PathFollowerAction(intakeTrenchPath));
        //lineup turret action
        runAction(new setTurretAction());
        //run action - shoot   
        runAction(new SpeedUpShooterAction(targetDistance));
        runAction(new FeedBallsAction(ballCount));
        runAction(new WaitAction(doneShooting));
        runAction(new FeedBallsAction(ballNumber));
        runAction(new SpeedUpShooterAction(shooterStop)); 
                 
    }
}
