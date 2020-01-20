/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.command_status.RobotState;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.sensors.NavX;
import frc.robot.lib.util.DataLogController;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Pose;
import frc.robot.loops.LoopController;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.VisionDriveAssistant;
import frc.robot.vision.VisionTargetList;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  SelectedDriverControls selectedDriverControls = SelectedDriverControls.getInstance();
  private AutoModeExecuter autoModeExecuter = null;
  private LoopController loopController;

 	RobotState robotState = RobotState.getInstance();
	Drive drive = Drive.getInstance();
	VisionTargetList visionTargetList = VisionTargetList.getInstance();
	VisionDriveAssistant visionDriveAssistant = VisionDriveAssistant.getInstance();
  Limelight camera = Limelight.getInstance();

  Shooter shooter;
  ControlPanel controlPanel;
  //Intake intake;
 // Agitator agitator;
  SmartDashboardInteractions smartDashboardInteractions = SmartDashboardInteractions.getInstance();

  DataLogController robotLogger;
  OperationalMode operationalMode = OperationalMode.getInstance();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    shooter = Shooter.getInstance();
    controlPanel = ControlPanel.getInstance();
    //intake = Intake.getInstance();
    //agitator = Agitator.getInstance();


    loopController = new LoopController();
    // loopController.register(drive.getVelocityPIDLoop());
    // loopController.register(DriveLoop.getInstance());
    // loopController.register(RobotStateLoop.getInstance());
    // loopController.register(VisionLoop.getInstance());
    // loopController.register(GoalStateLoop.getInstance());
    loopController.register(Shooter.getInstance());
    //loopController.register(Intake.getInstance());
    //loopController.register(Agitator.getInstance()); //Agitator is not yet set up with the loop interface


    selectedDriverControls.setDriverControls( smartDashboardInteractions.getDriverControlsSelection() );
    shooter = Shooter.getInstance();
    SmartDashboard.putNumber("Shooter/RPM", 0);
    SmartDashboard.putBoolean("Shooter/Debug", false);
    SmartDashboard.putBoolean("Shooter/UpdatePID", false);
    SmartDashboard.putNumber("Shooter/Debug/kP", 0);
    SmartDashboard.putNumber("Shooter/Debug/I", 0);
    SmartDashboard.putNumber("Shooter/Debug/kD", 0);
    SmartDashboard.putNumber("Agitator/Degree", 0);
    SmartDashboard.putBoolean("ControlPanel/Debug", false);
    SmartDashboard.putBoolean("Agitator/Debug", false);
    controlPanel.setupColors();

    robotLogger = DataLogController.getRobotLogController();
    robotLogger.register(this.getLogger());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
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

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
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

    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
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
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }



  @Override
  public void teleopInit() {
    loopController.start();
    camera.teleopInit();
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //loopController.run(); //Check LoopController run() function. Might be preventing systems from running do to if(!_running)....


    // SmartDashboard.putNumber("Gyro reading", NavX.getInstance().getHeadingDeg());
    // if(Limelight.getInstance().getPipeline() !=0){
    //   Limelight.getInstance().setPipeline(0);
    // }

    // selectedDriverControls.setDriverControls(smartDashboardInteractions.getDriverControlsSelection());
    // SelectedDriverControls driverControls = SelectedDriverControls.getInstance();

    // boolean visionButton = selectedDriverControls.getBoolean(DriverControlsEnum.DRIVE_ASSIST);

		// DriveCommand driveCmd = selectedDriverControls.getDriveCommand();
		// driveCmd = visionDriveAssistant.assist(driveCmd, visionButton);

		// //modify drive controls based on buttons
		// drive.setOpenLoop(driveCmd);



    shooter.run();
    controlPanel.run();



    //agitator.run();
   // intake.run();

  //   if (SmartDashboard.getBoolean("Agitator/Debug", false))
  //   {
  //     double agitatorSet = SmartDashboard.getNumber("Agitator/Degree", 0);
  //     agitator.setDegree(agitatorSet);
  //   }
  //   if (driverControls.getBoolean(DriverControlsEnum.DRIVE_ASSIST)||driverControls.getBoolean(DriverControlsEnum.SHOOT))
  //   {
  //     camera.setLEDMode(LedMode.kOn);
  //   }
  //   else
  //   {
  //     camera.setLEDMode(LedMode.kOff);
  //   }
  }

  

  @Override
  public void testPeriodic() {
  }


  public void setInitialPose (Pose _initialPose)
  {
    robotState.reset(_initialPose);
    System.out.println("InitialPose: " + _initialPose);
  }
  
  public void zeroAllSensors()
  {
    drive.zeroSensors();
  //  agitator.zeroSensors();
  }
  
  public void stopAll()
  {
    drive.stop();
   // intake.stop();
   // agitator.stop();
    shooter.stop();
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
