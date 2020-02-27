package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.sensors.Pigeon;
import frc.robot.lib.util.DataLogController;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.FallingEdgeDetector;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.loops.LoopController;
import frc.robot.vision.VisionDriveAssistant;
import frc.robot.loops.DriveLoop;
import frc.robot.subsystems.*;



public class Robot extends TimedRobot {
  SelectedDriverControls selectedDriverControls = SelectedDriverControls.getInstance();
  private AutoModeExecuter autoModeExecuter = null;
  private LoopController loopController;

	VisionDriveAssistant visionDriveAssistant = VisionDriveAssistant.getInstance();
  Limelight camera = Limelight.getInstance();
  Pigeon pigeon = (Pigeon)Pigeon.getInstance();

  //Drive drive = Drive.getInstance();
  //ControlPanel controlPanel;

  SmartDashboardInteractions smartDashboardInteractions = SmartDashboardInteractions.getInstance();

  DataLogController robotLogger;
  OperationalMode operationalMode = OperationalMode.getInstance();

  boolean testLoopCheck = false;
  FallingEdgeDetector testLoopEdge = new FallingEdgeDetector();


  @Override
  public void robotInit() {
    //controlPanel = ControlPanel.getInstance();

    loopController = new LoopController();
    // loopController.register(DriveLoop.getInstance());
    // loopController.register(Intake.getInstance());
    // loopController.register(ConveyorBelt.getInstance());
    loopController.register(Shooter.getInstance());
    // loopController.register(Lift.getInstance());


    selectedDriverControls.setDriverControls( smartDashboardInteractions.getDriverControlsSelection() );

    SmartDashboard.putNumber("Shooter/Debug/SetShooterRPM", 0);
    SmartDashboard.putNumber("Shooter/Debug/SetTurretDeg", 0);
    SmartDashboard.putNumber("Shooter/Debug/SetHoodDeg", 0);

    SmartDashboard.putBoolean("Shooter/Debug", true);
    SmartDashboard.putBoolean("Shooter/UpdatePID", false);
    SmartDashboard.putBoolean("ControlPanel/Debug", false);
    //controlPanel.setupColors();

    robotLogger = DataLogController.getRobotLogController();
    robotLogger.register(this.getLogger());
  }



  @Override
  public void robotPeriodic() {
    //loopController.run();	
    if(testLoopEdge.update(testLoopCheck)){
      Shooter.getInstance().resetForCalibration();
    }
    testLoopCheck = false;
  }



  @Override
	public void disabledInit()
	{
		operationalMode.set(OperationalMode.OperationalModeEnum.DISABLED);
		boolean logToFile = false;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);
		zeroAllSensors();

		Shuffleboard.stopRecording();

			if (autoModeExecuter != null)
			{
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;

			stopAll(); // stop all actuators
			loopController.start();
  }
  


	@Override
	public void disabledPeriodic()
	{
			stopAll(); // stop all actuators

			camera.disabledPeriodic();
	}



  @Override
  public void autonomousInit() {
    operationalMode.set(OperationalMode.OperationalModeEnum.AUTONOMOUS);
    boolean logToFile = false;
    boolean logToSmartDashboard = true;
    robotLogger.setOutputMode(logToFile, logToSmartDashboard);

    pigeon.zeroHeading(smartDashboardInteractions.getSelectedStartPose().getHeadingDeg()); //Setting up gyro with initial conditions

    loopController.start();
		Shuffleboard.startRecording();

    camera.autoInit();

    selectedDriverControls.setDriverControls(smartDashboardInteractions.getDriverControlsSelection());

    if (autoModeExecuter != null)
    {
      autoModeExecuter.stop();
    }
    autoModeExecuter = null;
    autoModeExecuter = new AutoModeExecuter();
    autoModeExecuter.setAutoMode(smartDashboardInteractions.getAutoModeSelection());
    autoModeExecuter.start();
  }
  


  @Override
  public void autonomousPeriodic() {
  }



  @Override
  public void teleopInit() {
    loopController.start();
    camera.teleopInit();
  }
  


  @Override
  public void teleopPeriodic() {
    loopController.run(); //To run the majority of the subsystems
    //drive.setOpenLoop(selectedDriverControls.getDriveCommand());

    //controlPanel.run();
  }

  

  @Override
  public void testPeriodic() {
    //Calibrating shooter!
    Shooter.getInstance().calibrate();
    testLoopCheck = true;
  }
  
  public void zeroAllSensors()
  {
    //drive.zeroSensors();
  }
  
  public void stopAll()
  {
    loopController.stop();
    //drive.stop(); //for now
  }


  private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			    put("OperationalMode", operationalMode.get().toString());
        }
    };
    
    public DataLogger getLogger() { return logger; }

}
