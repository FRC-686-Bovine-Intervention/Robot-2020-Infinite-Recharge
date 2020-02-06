package frc.robot.auto.actions;

import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Vector2d;
import frc.robot.subsystems.Shooter;

public class AimShooterAction implements Action {
    //This action is used to line the turret up with the shooter
    //

    
    private boolean finished = false;

    private Shooter shooter = Shooter.getInstance();
    private Limelight limelight = Limelight.getInstance();

    private double backupAngle; //This is used in the event that a target can not be found
    private int sensingAttempts = 0; //Tracks attempts
    private static int maxAttempts = 5;

    private static double pCorrection = 0.5; //Used for angle correction. Proportional
    private static double toleranceRads = 0.075; //Allowable error in radians
    public Vector2d lastTargetPos = null;

    
    public AimShooterAction(double suggestedAngle) {
        backupAngle = suggestedAngle;
    }



    @Override
    public void start() {
        shooter.setTurretDeg(backupAngle); //Start by looking in the general direction
    }

    @Override
    public void update() {
        if(!limelight.getIsTargetFound()){
            sensingAttempts++;
            if(sensingAttempts >= maxAttempts){
                finished = true; //Exit action
                return;
            }
        } else {
            Vector2d targetPos = shooter.getTargetDisplacement();
            lastTargetPos = targetPos; //Updating this for external functions
            //First see if we are close enough:
            double cTurretAngle = shooter.getTurretAngleRad();
            double errorRad = targetPos.angle()-(cTurretAngle+(Math.PI/2.0)); //Should output positive if CCW correction is needed
            if(Math.abs(errorRad) <= toleranceRads){
                finished = true;
                return; //Exit action
            }
            //Otherwise...
            double correction = errorRad*pCorrection; //Assumes positive means leftwards correction
            shooter.setTurretDeg(Math.toDegrees(cTurretAngle+correction)); //Adjusting turret
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {

    }

    public Vector2d getSensedTargetPos(){
        return lastTargetPos; //In the event that we want to reuse the 
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