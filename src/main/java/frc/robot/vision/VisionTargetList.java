package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import frc.robot.lib.util.DataLogger;

public class VisionTargetList
{
	/*
	 * The VisionTargetListState keeps track of multiple targets
	 */

	private static VisionTargetList instance = new VisionTargetList();
	public static VisionTargetList getInstance() { return instance; }	

	private List<Target> targets = new ArrayList<>();
	private double imageCaptureTimestamp = 0;
	
	
	// Synchronized get/set functions for access from other threads
	
	synchronized public void set(double _imageCaptureTimestamp, List<Target> _targets)
	{ 
		imageCaptureTimestamp = _imageCaptureTimestamp;
		targets = _targets; 
	}
	synchronized public List<Target> getTargets() 			{ return targets; }
	synchronized public double getImageCaptureTimestamp()	{ return imageCaptureTimestamp; }


	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
			int k=0;
			for (Target target : targets)
			{
                put(String.format("VisionTargetList/Target%d/hCenter", k), target.hCenter);
                put(String.format("VisionTargetList/Target%d/hWidth",  k), target.hWidth);
                put(String.format("VisionTargetList/Target%d/vCenter", k), target.vCenter);
                put(String.format("VisionTargetList/Target%d/vWidth",  k), target.vWidth);
				k++;
			}
		}
	};

	public DataLogger getLogger()
	{
		return logger;
	}


	public static class Target
	{
	/*
	 * A container class for Targets detected by the vision system, containing the
	 * horizontal and vertical angles from the optical axis.
	 */
		protected double hCenter; 	// horizontal angle to center of target from optical axis, in radians
		protected double vCenter; 	//   vertical angle to center of target from optical axis, in radians

		protected double hWidth; 	// horizontal angular width of target, in radians
		protected double vWidth; 	//   vertical angular width of target, in radians

		public Target(double _hCenter, double _vCenter, double _hWidth, double _vWidth)
		{
			hCenter = _hCenter;
			vCenter = _vCenter;
			hWidth = _hWidth;
			vWidth = _vWidth;
		}

		public double getHorizontalAngle()	{ return hCenter; }
		public double getVerticalAngle()	{ return vCenter; }
		public double getHorizontalWidth()	{ return hWidth; }
		public double getVerticalWidth()	{ return vWidth; }

		public String toString()
		{
			return "hCenter:" + hCenter + ", hWidth:" + hWidth + ", vCenter:" + vCenter + ", vWidth:" + vWidth + ".";
		}
	}
}