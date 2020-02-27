package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CalibrateAction;

public class CalibrateAuto extends AutoModeBase {

    //Literally just calibrates the robot.

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new CalibrateAction());
    }
}