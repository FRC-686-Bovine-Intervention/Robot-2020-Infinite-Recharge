// package frc.robot.auto.actions;

// import frc.robot.command_status.DriveCommand;
// import frc.robot.lib.sensors.Limelight;
// import frc.robot.lib.util.DataLogger;
// import frc.robot.lib.util.Vector2d;
// import frc.robot.subsystems.Drive;
// import frc.robot.vision.VisionDriveAssistant;

// public class VisionTurretAction implements Action {



//     private double power;
//     private boolean finished = false;
//     private Shooter shooter = Shooter.getInstance();
//     private Shooter visionCommand;
//     private VisionDriveAssistant visionDriveAssistant;
//     double speed = 0.25;
//     double stopCameraAngleThreshold = -10; //Was -9


//     public VisionTurretAction(double power){
//         this.power = power;
//     }

//     @Override
//     public void start() {
//         Limelight.getInstance().setLEDMode(Limelight.LedMode.kOn);
//         Limelight.getInstance().setPipeline(1);
//         visionDriveAssistant = VisionDriveAssistant.getInstance();
//         visionCommand = new shooter.setTurretDeg(0 ,0);
//     }

//     @Override
//     public void update() {
//         visionCommand = new DriveCommand(speed, speed);
//         visionCommand = visionDriveAssistant.assist(visionCommand, true);
//         shooter.setOpenLoop(visionCommand);
//         Limelight camera = Limelight.getInstance();
//         finished = (camera.getIsTargetFound() && ((camera.getTargetVerticalAngleRad()*Vector2d.radDeg) > stopCameraAngleThreshold));
//     }

//     @Override
//     public boolean isFinished() {
//         return finished;
//     }

//     @Override
//     public void done() {
//     }

//     private final DataLogger logger = new DataLogger()
//     {
//         @Override
//         public void log()
//         {
// 	    }
//     };
	
//     public DataLogger getLogger() { return logger; }
// }