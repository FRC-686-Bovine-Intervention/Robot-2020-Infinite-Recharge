package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.Shooter;

public class SpeedUpShooterAction implements Action {
    //This function determines the necessary speed for the shooter and gets it up to speed
    private Shooter shooter = Shooter.getInstance();
    private ConveyorBelt conveyorBelt = ConveyorBelt.getInstance();

    private boolean finished = false;

    private double targetDistance;
    private double targetRPM;
    private double startTime;
    private static final double reverseTime = 1.0, minimumTime = reverseTime+0.5;



    public SpeedUpShooterAction(double targetDistance){
        this.targetDistance = targetDistance;
    }

    @Override
    public void start() {
        targetRPM = shooter.getTargetShooterVelocity(targetDistance)*(30.0/Math.PI);
        shooter.setShooterRPM(targetRPM);

        conveyorBelt.runKicker();
        conveyorBelt.reverseTower();


        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        double elapsedTime = Timer.getFPGATimestamp()-startTime;
        if(elapsedTime >=reverseTime){
            conveyorBelt.stopTower();
        }
        if(shooter.nearTarget(true) && elapsedTime >= minimumTime){
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