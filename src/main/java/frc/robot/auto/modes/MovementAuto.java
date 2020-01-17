package frc.robot.auto.modes;


import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CurvedDriveAction;
import frc.robot.auto.actions.TimedDriveStraightAction;
import frc.robot.auto.actions.VisionDriveAction;

public class MovementAuto extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new CurvedDriveAction(-0.3, 31.25, -90));
        runAction(new CurvedDriveAction(-0.3, -31.25, 30));
        runAction(new VisionDriveAction(0.25));
	}

}