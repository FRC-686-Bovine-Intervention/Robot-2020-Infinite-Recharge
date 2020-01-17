package frc.robot.auto.modes;

import frc.robot.SmartDashboardInteractions;
import frc.robot.SmartDashboardInteractions.*;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CurvedDriveAction;
import frc.robot.auto.actions.DriveToAngle;
import frc.robot.auto.actions.LimelightLEDAction;
import frc.robot.auto.actions.SetShooterSpeedAction;
import frc.robot.auto.actions.ShootBallAction;
import frc.robot.auto.actions.StopShooterAction;
import frc.robot.auto.actions.VisionDriveAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.sensors.Limelight.LedMode;
import frc.robot.subsystems.Shooter;

public class BunnybotsAuto extends AutoModeBase {

    private AutoSideSelection side;
    private SmartDashboardInteractions smartDash = SmartDashboardInteractions.getInstance();

    public BunnybotsAuto() 
    { 
    }
    
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Start BunnybotsAuto");
        side = smartDash.getAutoSide();

        // Turn on Limelight
        runAction(new LimelightLEDAction(LedMode.kOn));

//Start Here
        
        // Shoot at high goal
        System.out.println("Set Shooter Speed for High Goal");
        Limelight.getInstance().setPipeline(0);
        runAction(new SetShooterSpeedAction(Shooter.GoalEnum.HIGH_GOAL));
        System.out.println("Shoot 5 Balls");
        int numBalls = 5;
        runAction(new ShootBallAction(numBalls));
        System.out.println("Wait for Ball to Shoot");
        runAction(new WaitAction(2.0));     // wait for  motor to change speeds


        // this is where we move to the low goal
        // if(side == AutoSideSelection.LEFT_SIDE){
        //     runAction(new CurvedDriveAction(-0.3, 31.25, -90));
        //     runAction(new CurvedDriveAction(-0.3, -31.25, 30));
        // }
        // if(side == AutoSideSelection.RIGHT_SIDE){
        //     runAction(new CurvedDriveAction(-0.3, -31.25, 90));
        //     runAction(new CurvedDriveAction(-0.3, 31.25,  -10));
        // }
        
        
        // Limelight.getInstance().setPipeline(1);
        // runAction(new WaitAction(0.5)); 
        // //Approaching Low goal
        // runAction(new VisionDriveAction(0.25));

        // // Shoot at low goal
        // System.out.println("Set Shooter Speed for Low Goal");
        // runAction(new SetShooterSpeedAction(Shooter.GoalEnum.LOW_GOAL));
        // System.out.println("Shoot 2 Balls");
        // numBalls = 2;
        // runAction(new ShootBallAction(numBalls));
        // System.out.println("Wait for Ball to Shoot");
        // runAction(new WaitAction(2.0));     // wait for  motor to change speeds

        System.out.println("Stop Shooter");
        runAction(new StopShooterAction());
        
        // Turn off Limelight
        runAction(new LimelightLEDAction(LedMode.kOff));
    
        //Backing into Neutral Zone
        runAction(new DriveToAngle(-0.5, 0, 3.5));
    }
}