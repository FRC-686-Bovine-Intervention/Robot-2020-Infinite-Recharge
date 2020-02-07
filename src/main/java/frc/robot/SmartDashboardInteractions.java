package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.modes.FieldDimensions;
import frc.robot.auto.modes.MovementAuto;
import frc.robot.auto.modes.StandStillMode;
import frc.robot.lib.joystick.DriverControlsBase;
import frc.robot.lib.joystick.DriverControlsReversibleThrustmaster;
import frc.robot.lib.joystick.DriverControlsThrustmaster;
import frc.robot.lib.joystick.DriverControlsThrustmasterRight;
import frc.robot.lib.joystick.DriverControlsXbox;
import frc.robot.lib.util.Pose;

/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class SmartDashboardInteractions 
{
	private static SmartDashboardInteractions instance = null;

	public static SmartDashboardInteractions getInstance() {
		if (instance == null) {
			instance = new SmartDashboardInteractions();
		}
		return instance;
    }
        
    public SmartDashboardInteractions()
    {
        initWithDefaults();
    }


    public void initWithDefaults() 
    {
    	driverControlsChooser = new SendableChooser<DriverControlsOption>();
    	driverControlsChooser.addOption(DriverControlsOption.XBOX_ARCADE.name,        DriverControlsOption.XBOX_ARCADE);
        driverControlsChooser.setDefaultOption(DriverControlsOption.THRUSTMASTER_RIGHTHANDED.name,  DriverControlsOption.THRUSTMASTER_RIGHTHANDED);
        driverControlsChooser.addOption(DriverControlsOption.THRUSTMASTER_RIGHTHANDED.name, DriverControlsOption.THRUSTMASTER_RIGHTHANDED);
        driverControlsChooser.addOption(DriverControlsOption.THRUSTMASTER_REVERSIBLE.name, DriverControlsOption.THRUSTMASTER_REVERSIBLE);
    	// driverControlsChooser.addOption(DriverControlsOption.ARCADE.name,        DriverControlsOption.ARCADE);
		// driverControlsChooser.addOption(DriverControlsOption.TRIGGER.name,        DriverControlsOption.TRIGGER);
    	// driverControlsChooser.addOption(DriverControlsOption.TANK.name, 	      DriverControlsOption.TANK);
     	// driverControlsChooser.addOption(DriverControlsOption.CHEESY_ARCADE.name,  DriverControlsOption.CHEESY_ARCADE);
    	// driverControlsChooser.addOption(DriverControlsOption.CHEESY_TRIGGER.name, DriverControlsOption.CHEESY_TRIGGER);
        // driverControlsChooser.addOption(DriverControlsOption.CHEESY_2STICK.name,  DriverControlsOption.CHEESY_2STICK);
        // driverControlsChooser.addOption(DriverControlsOption.THRUSTMASTER_ARCADE.name,  DriverControlsOption.THRUSTMASTER_ARCADE);
        // driverControlsChooser.addOption(DriverControlsOption.THRUSTMASTER_TANK.name,  DriverControlsOption.THRUSTMASTER_TANK);
        // driverControlsChooser.addOption(DriverControlsOption.THRUSTMASTER_2STICK.name,  DriverControlsOption.THRUSTMASTER_2STICK);
    	SmartDashboard.putData("Driver Controls", driverControlsChooser);



        startDelayChooser = new SendableChooser<StartDelayOption>();
        startDelayChooser.setDefaultOption(StartDelayOption.DELAY_0_SEC.toString(), StartDelayOption.DELAY_0_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_1_SEC.toString(), StartDelayOption.DELAY_1_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_2_SEC.toString(), StartDelayOption.DELAY_2_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_3_SEC.toString(), StartDelayOption.DELAY_3_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_4_SEC.toString(), StartDelayOption.DELAY_4_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_5_SEC.toString(), StartDelayOption.DELAY_5_SEC);
        SmartDashboard.putData("Auto Start Delay", startDelayChooser);


        autoModeChooser = new SendableChooser<AutoModeOption>();
        autoModeChooser.setDefaultOption(AutoModeOption.MOVEMENT_AUTO.toString(), AutoModeOption.MOVEMENT_AUTO);
        autoModeChooser.addOption(AutoModeOption.MOVEMENT_AUTO.toString(), AutoModeOption.MOVEMENT_AUTO);
        SmartDashboard.putData("Auto Selection", autoModeChooser);

        // autoSideChooser = new SendableChooser<AutoSideSelection>();
        // autoSideChooser.setDefaultOption(AutoSideSelection.LEFT_SIDE.toString(), AutoSideSelection.LEFT_SIDE);
        // autoSideChooser.addOption(AutoSideSelection.RIGHT_SIDE.toString(), AutoSideSelection.RIGHT_SIDE);
        // autoSideChooser.addOption(AutoSideSelection.LEFT_SIDE.toString(), AutoSideSelection.LEFT_SIDE);
        // SmartDashboard.putData("Side Selection", autoSideChooser);

        //Choosing start position
        startPoseChooser = new SendableChooser<StartPoseSelection>();
        startPoseChooser.addOption(StartPoseSelection.PORT_START.name, StartPoseSelection.PORT_START);
        startPoseChooser.addOption(StartPoseSelection.MIDDLE_START.name, StartPoseSelection.MIDDLE_START);
        startPoseChooser.addOption(StartPoseSelection.PLAYER_STATION.name, StartPoseSelection.PLAYER_STATION);
        startPoseChooser.setDefaultOption(StartPoseSelection.PORT_START.name, StartPoseSelection.PORT_START);
        SmartDashboard.putData("Start Pose Selection", startPoseChooser);


        
    }

        
    
    
    SendableChooser<DriverControlsOption> driverControlsChooser;
    
    enum DriverControlsOption 
    {
        XBOX_ARCADE("Xbox  Arcade"),
        THRUSTMASTER_ARCADE("Thrustmaster  Arcade"),
        THRUSTMASTER_RIGHTHANDED("Right Thrustmaster Arcade"),
        THRUSTMASTER_REVERSIBLE("Thrustmaster Reversible");
        // ARCADE("Arcade"),
        // TRIGGER("Trigger"),				// works for Xbox controller and Xbox steering wheel
        // TANK("Tank"),
        // CHEESY_ARCADE("Cheesy Arcade"),
        // CHEESY_TRIGGER("Cheesy Trigger"),
        // CHEESY_2STICK("Cheesy Two-Stick"),
        // THRUSTMASTER_ARCADE("Thrustmaster Arcade"),
        // THRUSTMASTER_TANK("Thrustmaster Tank"),
        // THRUSTMASTER_2STICK("Thrustmaster Two-Stick");

    	public final String name;
    	
        DriverControlsOption(final String name) {
    		this.name= name;
    	}
    }

    public DriverControlsBase getDriverControlsSelection() {
        DriverControlsOption selection = (DriverControlsOption) driverControlsChooser.getSelected();
        if (selection == null) {
            selection = DriverControlsOption.THRUSTMASTER_ARCADE;
        }

        switch (selection) {
        case XBOX_ARCADE:
            return new DriverControlsXbox();

        case THRUSTMASTER_RIGHTHANDED:
            return new DriverControlsThrustmasterRight();

        case THRUSTMASTER_ARCADE:
            return null;

        case THRUSTMASTER_REVERSIBLE:
            return new DriverControlsReversibleThrustmaster();
        
        default:
            return new DriverControlsThrustmaster();

        }
    }

    SendableChooser<StartDelayOption> startDelayChooser;

    public enum StartDelayOption {
        DELAY_0_SEC("0 Sec", 0.0), DELAY_1_SEC("1 Sec", 1.0), DELAY_2_SEC("2 Sec", 2.0), DELAY_3_SEC("3 Sec", 3.0),
        DELAY_4_SEC("4 Sec", 4.0), DELAY_5_SEC("5 Sec", 5.0);

        public final String name;
        public final double delaySec;

        StartDelayOption(final String name, final double delaySec) {
            this.name = name;
            this.delaySec = delaySec;
        }
    }

    public double getStartDelay() {
        return startDelayChooser.getSelected().delaySec;
    }

    SendableChooser<AutoModeOption> autoModeChooser;

    enum AutoModeOption {
        MOVEMENT_AUTO("Movement Auto");

        public final String name;

        AutoModeOption(final String name) {
            this.name = name;
        }
    }

    public AutoModeBase getAutoModeSelection() {
        final AutoModeOption autoMode = (AutoModeOption) autoModeChooser.getSelected();

        switch (autoMode) {
        case MOVEMENT_AUTO:
            return new MovementAuto();

        default:
            System.out.println("ERROR: unexpected auto mode: " + autoMode);
            return new StandStillMode();
        }
    }

    // SendableChooser<AutoSideSelection> autoSideChooser;

    // public enum AutoSideSelection {

    //     // Left Looking at the goals
    //     LEFT_SIDE("Left Side"), RIGHT_SIDE("Right Side");

    //     public final String selection;

    //     AutoSideSelection(final String selection) {
    //         this.selection = selection;
    //     }
    // }

    // public AutoSideSelection getAutoSide() {
    //     final AutoSideSelection selection = (AutoSideSelection) autoSideChooser.getSelected();
    //     return selection;
    // }
    


    SendableChooser<StartPoseSelection> startPoseChooser;
    public enum StartPoseSelection {
        PORT_START("Port Position", FieldDimensions.portStartPose),
        MIDDLE_START("Middle Position", FieldDimensions.middleStartPose),
        PLAYER_STATION("Player Station", FieldDimensions.playerStationStartPose);


        public final String name;
        public final Pose pose;

        StartPoseSelection(final String name, final Pose pose){
            this.name = name;
            this.pose = pose;
        }
    }

    public Pose getSelectedStartPose(){
        return startPoseChooser.getSelected().pose;
    }
}
   


