package frc.robot.auto.actions;

import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Shooter;

public class SpeedUpShooterAction implements Action {
    //This function determines the necessary speed for the shooter and gets it up to speed
    private Shooter shooter = Shooter.getInstance();

    private boolean finished = false;

    private double targetDistance;
    private double targetRPM;



    public SpeedUpShooterAction(double targetDistance){
        this.targetDistance = targetDistance;
    }

    @Override
    public void start() {
        targetRPM = shooter.getTargetShooterVelocity(targetDistance)*(30.0/Math.PI);
        shooter.setShooterRPM(targetRPM);
    }

    @Override
    public void update() {
        if(shooter.nearTarget(true)){
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub

    }

    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
	    }
    };
	
	@Override
	public DataLogger getLogger() { return logger; }

}