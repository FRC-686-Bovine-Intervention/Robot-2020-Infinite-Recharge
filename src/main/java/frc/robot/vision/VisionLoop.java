package frc.robot.vision;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.sensors.Limelight.BoundingAngles;
import frc.robot.lib.sensors.Limelight.BoundingRectangle;
import frc.robot.lib.util.DataLogger;
import frc.robot.loops.Loop;

/**
 * VisionLoop contains the various attributes calculated by the vision system,
 * namely a list of targets and the timestamp at which it was captured.
 */
public class VisionLoop implements Loop {
	private static VisionLoop instance = new VisionLoop();

	public static VisionLoop getInstance() {
		return instance;
	}

	// camera selection
	public Limelight camera = Limelight.getInstance();
	Limelight cameraSelection = camera;

	public VisionTargetList visionTargetList = VisionTargetList.getInstance();

	BoundingRectangle boundingRectangle = cameraSelection.new BoundingRectangle();

	@Override
	public void onStart() {
		// nothing
	}

	@Override
	public void onLoop() {
		double currentTime = Timer.getFPGATimestamp();

		// get target info from Limelight
		getTargets(currentTime);
	}

	@Override
	public void onStop() {
		// nothing
	}

	public void getTargets(double currentTime) {
		cameraSelection = camera;

		double cameraLatency = cameraSelection.getTotalLatencyMs() / 1000.0;
		double imageCaptureTimestamp = currentTime - cameraLatency; // assumes transport time from phone to this code is
																	// instantaneous

		ArrayList<VisionTargetList.Target> targets = new ArrayList<>();	// initially empty

		if (cameraSelection.getIsTargetFound()) 
		{
			boundingRectangle = cameraSelection.getBoundingRectangle();

			if (boundingRectangle.xMin > 0 && boundingRectangle.xMax < (Limelight.kImageWidthPixels-1) && 
			    boundingRectangle.yMin > 0 && boundingRectangle.yMax < (Limelight.kImageHeightPixels-1))
			{
				// no corners at limits (indicating we are too close, and should just use a previous value)					
				BoundingAngles boundingAngles = cameraSelection.getBoundingAnglesRad(boundingRectangle);

				double hAngle = cameraSelection.getTargetHorizontalAngleRad();
				double vAngle = cameraSelection.getTargetVerticalAngleRad();
				double hWidth = boundingAngles.hWidthRad;
				double vWidth = boundingAngles.vWidthRad;

				VisionTargetList.Target target = new VisionTargetList.Target(hAngle, vAngle, hWidth, vWidth);
				targets.add(target);
			}
		}

		visionTargetList.set(imageCaptureTimestamp, targets);
	}


	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			put("VisionLoop/selectedCamera", cameraSelection.getTableName());
			put("VisionLoop/isTargetFound", cameraSelection.getIsTargetFound());
			put("VisionLoop/Corners.xMin", boundingRectangle.xMin);
			put("VisionLoop/Corners.xMax", boundingRectangle.xMax);
			put("VisionLoop/Corners.yMin", boundingRectangle.yMin);
			put("VisionLoop/Corners.yMax", boundingRectangle.yMax);
        }
    };
    
    public DataLogger getLogger() { return logger; }



	/**
	 * @return the cameraSelection
	 */
	public Limelight getCameraSelection() {
		return cameraSelection;
	}

	/**
	 * @param cameraSelection the cameraSelection to set
	 */
	public void setCameraSelection(Limelight cameraSelection) {
		this.cameraSelection = cameraSelection;
	}

	/**
	 * @return the visionTargetList
	 */
	public VisionTargetList getVisionTargetList() {
		return visionTargetList;
	}

	/**
	 * @param visionTargetList the visionTargetList to set
	 */
	public void setVisionTargetList(VisionTargetList visionTargetList) {
		this.visionTargetList = visionTargetList;
	}

	/**
	 * @return the boundingRectangle
	 */
	public BoundingRectangle getBoundingRectangle() {
		return boundingRectangle;
	}

	/**
	 * @param boundingRectangle the boundingRectangle to set
	 */
	public void setBoundingRectangle(BoundingRectangle boundingRectangle) {
		this.boundingRectangle = boundingRectangle;
	}


}
