package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.sensors.NavX;
import frc.robot.lib.util.DataLogController;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Pose;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.LoopController;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.VisionDriveAssistant;
import frc.robot.vision.VisionTargetList;



public class Robot extends TimedRobot {
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  SelectedDriverControls selectedDriverControls = SelectedDriverControls.getInstance();
  private AutoModeExecuter autoModeExecuter = null;
  private LoopController loopController;

	VisionTargetList visionTargetList = VisionTargetList.getInstance();
	VisionDriveAssistant visionDriveAssistant = VisionDriveAssistant.getInstance();
  Limelight camera = Limelight.getInstance();

  //Drive drive = Drive.getInstance();
  //ControlPanel controlPanel;

  SmartDashboardInteractions smartDashboardInteractions = SmartDashboardInteractions.getInstance();

  DataLogController robotLogger;
  OperationalMode operationalMode = OperationalMode.getInstance();


  @Override
  public void robotInit() {
    //controlPanel = ControlPanel.getInstance();

    loopController = new LoopController();
    // loopController.register(DriveLoop.getInstance());
    // loopController.register(Intake.getInstance());
    // loopController.register(Conveyor.getInstance());
    loopController.register(Shooter.getInstance());
    // loopController.register(Lift.getInstance());

    selectedDriverControls.setDriverControls( smartDashboardInteractions.getDriverControlsSelection() );
    SmartDashboard.putNumber("Shooter/RPM", 0);
    SmartDashboard.putNumber("Shooter/HoodDegree", 0);
    SmartDashboard.putBoolean("Shooter/Debug", false);
    SmartDashboard.putBoolean("Shooter/UpdatePID", false);
    SmartDashboard.putNumber("Shooter/Debug/kP", 0);
    SmartDashboard.putNumber("Shooter/Debug/I", 0);
    SmartDashboard.putNumber("Shooter/Debug/kD", 0);
    SmartDashboard.putNumber("Agitator/Degree", 0);
    SmartDashboard.putBoolean("ControlPanel/Debug", false);
    SmartDashboard.putBoolean("Agitator/Debug", false);
    SmartDashboard.putNumber("Shooter/TargetDist", 0);
    //controlPanel.setupColors();

    robotLogger = DataLogController.getRobotLogController();
    robotLogger.register(this.getLogger());
  }



  @Override
  public void robotPeriodic() {
    loopController.run();	
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

    NavX.getInstance().zeroSensor();

    loopController.start();
		Shuffleboard.startRecording();

    camera.autoInit();

    m_autoSelected = m_chooser.getSelected();
    selectedDriverControls.setDriverControls( smartDashboardInteractions.getDriverControlsSelection() );

    System.out.println("Auto selected: " + m_autoSelected);
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
    drive.setOpenLoop(selectedDriverControls.getDriveCommand());

    //controlPanel.run();
  }

  

  @Override
  public void testPeriodic() {
  }


  public void setInitialPose (Pose _initialPose)
  {
    System.out.println("InitialPose: " + _initialPose);
  }
  
  public void zeroAllSensors()
  {
    drive.zeroSensors();
  }
  
  public void stopAll()
  {
    loopController.stop();
    drive.stop();
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
