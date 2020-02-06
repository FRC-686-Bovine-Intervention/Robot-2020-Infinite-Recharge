package frc.robot.lib.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.Vector2d;

// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.HashMap;
import java.util.Map;

/**
 * Limelight Class was started by Corey Applegate of Team 3244 Granite City
 * Gearheads. We Hope you Enjoy the Limelight Camera.
 */
public class Limelight
{
    // Limelight versions
    public static final int V1 = 0;
    public static final int V2 = 1;

    private static Limelight instance = new Limelight("limelight", V1);
    public static Limelight getInstance() { return instance; }

    public static final double kImageHeightPixels = 240;
    public static final double kImageWidthPixels  = 320;

    public final double kImageVertCenterPixels  = (kImageHeightPixels-1.0)/2.0;
    public final double kImageHorizCenterPixels = (kImageWidthPixels-1.0)/2.0;

    // Limelight v1/v2 Horizontal and Vertical Field of View in Radians
    public final double kCameraVertFOVRad[]  = {41.0 * Vector2d.degreesToRadians, 49.7 * Vector2d.degreesToRadians};
    public final double kCameraHorizFOVRad[] = {54.0 * Vector2d.degreesToRadians, 59.6 * Vector2d.degreesToRadians};
    public final double kFOVError[] = {1.2, 1.6};   // empirically determined to give the correct range
    public double kCameraFocalLengthInPixels[] = {0, 0};

    public final double kImageCaptureLatencyMs = 11.0;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private NetworkTable table;
    private String tableName;
    private int version;

    /**
     * Using the Default Limelight NT table
     */
    public Limelight()
    {
        this("limelight", V2);
    }

    /**
     * If you changed the name of your Limelight tell me the new name
     */
    public Limelight(String _tableName, int _version)
    {
        tableName = _tableName;
        table = NetworkTableInstance.getDefault().getTable(tableName);
        version = _version;

        kCameraFocalLengthInPixels[V1] = kImageHorizCenterPixels / Math.atan(kCameraHorizFOVRad[V1]/2.0 * kFOVError[V1]);
        kCameraFocalLengthInPixels[V2] = kImageHorizCenterPixels / Math.atan(kCameraHorizFOVRad[V2]/2.0 * kFOVError[V2]);
    }

    /**
     * Helper funciton to configure limelight after booting. 
     * 
     * Call this function from disabledPeriodic() in Robot.java
     * (Limelight boots to slowly to be configured from robotInit() or disabledInit() )
     * Modify configuration as needed
     */
    public void disabledPeriodic()
    {
        boolean debuggingWhileDisabled = false;

        if (debuggingWhileDisabled)
        {
            setPipeline(0);
            setLEDMode(LedMode.kOn);
            setCamMode(CamMode.kVision);
            setSnapshot(Snapshot.kOff);
            setStream(StreamType.kStandard);
        }
        else
        {
            setPipeline(0);
            setLEDMode(LedMode.kOff);
            setCamMode(CamMode.kDriver);
            setSnapshot(Snapshot.kOff);
            setStream(StreamType.kPiPSecondary);
        }
    }

    /**
     * Helper funciton to configure limelight after booting. 
     * 
     * Call this function from autoInit() in Robot.java
     * Modify configuration as needed
     */
    public void autoInit()
    {
        setPipeline(0);
        setLEDMode(LedMode.kOn);
        setCamMode(CamMode.kVision);
        setSnapshot(Snapshot.kOff);
        setStream(StreamType.kPiPSecondary);
    }

    /**
     * Helper funciton to configure limelight after booting. 
     * 
     * Call this function from teleopInit() in Robot.java
     * Modify configuration as needed
     */
    public void teleopInit()
    {
        setPipeline(0);
        setLEDMode(LedMode.kOn);
        setCamMode(CamMode.kVision);
        setSnapshot(Snapshot.kOff);
        setStream(StreamType.kPiPSecondary);
    }

    public void LimelightInit()
    {
        // testAllTab();
    }
    // private void testAllTab(){
    // ShuffleboardTab LimeLightTab = Shuffleboard.getTab(m_tableName);
    // // To Do
    // // populate tab with all the data

    // }

    public boolean isConnected()
    {
        resetPipelineLatency();
        Timer.delay(.05); // How to make this not hold the thread?
        if (getPipelineLatency() == 0.0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    public String getTableName()
    {
        return tableName;
    }

    /**
     * tv Whether the limelight has any valid targets (0 or 1)
     * 
     * @return
     */
    public boolean getIsTargetFound()
    {
        NetworkTableEntry tv = table.getEntry("tv");
        double v = tv.getDouble(0);
        if (v == 0.0f)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
     * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     * 
     * @return
     */
    public double getTargetHorizontalAngleRad()
    {
        NetworkTableEntry tx = table.getEntry("tx");
        double x = -tx.getDouble(0.0);      // turns to left (negative x) result in positive theta
        return x * Vector2d.degreesToRadians;
    }

    /**
     * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     * 
     * @return
     */
    public double getTargetVerticalAngleRad()
    {
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y * Vector2d.degreesToRadians;
    }

    public double[] getXCorners()
    {
        return table.getEntry("tcornx").getDoubleArray(new double[0]);
    }

    public double[] getYCorners()
    {
        return table.getEntry("tcorny").getDoubleArray(new double[0]);
    }



    public class BoundingRectangle 
    {
		public double xMin, xMax, yMin, yMax;

		public BoundingRectangle()
		{
			xMin = Double.POSITIVE_INFINITY;
			xMax = Double.NEGATIVE_INFINITY;
			yMin = Double.POSITIVE_INFINITY;
			yMax = Double.NEGATIVE_INFINITY;
		}
	}

	public BoundingRectangle getBoundingRectangle()
	{
		BoundingRectangle boundingRectangle = new BoundingRectangle();

        double[] xCorn = getXCorners();
        double[] yCorn = getYCorners();

		for (int k=0; k<xCorn.length; k++)
		{
			boundingRectangle.xMin = Math.min(boundingRectangle.xMin, xCorn[k]);
			boundingRectangle.xMax = Math.max(boundingRectangle.xMax, xCorn[k]);
		}
		for (int k=0; k<yCorn.length; k++)
		{
			boundingRectangle.yMin = Math.min(boundingRectangle.yMin, yCorn[k]);
			boundingRectangle.yMax = Math.max(boundingRectangle.yMax, yCorn[k]);
		}
		return boundingRectangle;
	}

    public class BoundingAngles 
    {
		public double hWidthRad, vWidthRad;

		public BoundingAngles()
		{
			hWidthRad = 0.0;
			vWidthRad = 0.0;
		}
	}

    public BoundingAngles getBoundingAnglesRad(BoundingRectangle _boundingRectangle)
    {
        BoundingAngles boundingAngles = new BoundingAngles();
        boundingAngles.hWidthRad = horizPixelToAngleRad(_boundingRectangle.xMax) - horizPixelToAngleRad(_boundingRectangle.xMin);
        boundingAngles.vWidthRad =  vertPixelToAngleRad(_boundingRectangle.yMax) -  vertPixelToAngleRad(_boundingRectangle.yMin);
        return boundingAngles;
    }

    public double horizPixelToAngleRad(double _pixel)
    {
        return Math.atan( (_pixel-kImageHorizCenterPixels) / kCameraFocalLengthInPixels[version] );
    }

    public double vertPixelToAngleRad(double _pixel)
    {
        return Math.atan( (_pixel-kImageVertCenterPixels) / kCameraFocalLengthInPixels[version] );
    }

    /**
     * ta Target Area (0% of image to 100% of image)
     * 
     * @return
     */
    public double getTargetAreaPercentage()
    {
        NetworkTableEntry ta = table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }

    /**
     * ts Skew or rotation (-90 degrees to 0 degrees)
     * 
     * @return
     */
    public double getSkewRotation()
    {
        NetworkTableEntry ts = table.getEntry("ts");
        double s = ts.getDouble(0.0);
        return s;
    }

    /**
     * tl The pipeline’s latency contribution (ms) Add at least 11ms for image
     * capture latency.
     * 
     * @return
     */
    public double getPipelineLatency()
    {
        NetworkTableEntry tl = table.getEntry("tl");
        double l = tl.getDouble(0.0);
        return l;
    }

    private void resetPipelineLatency()
    {
        table.getEntry("tl").setValue(0.0);
    }

    public double getTotalLatencyMs()
    {
        // Limelight documentation says: 
        //      tl: The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
        return getPipelineLatency() + kImageCaptureLatencyMs;
    }

    // Setters

    /**
     * LedMode Sets limelight’s LED state
     * 
     * kon koff kblink
     * 
     * @param ledMode
     */
    public void setLEDMode(LedMode ledMode)
    {
        table.getEntry("ledMode").setValue(ledMode.getValue());
    }

    /**
     * Returns current LED mode of the Limelight
     * 
     * @return LedMode
     */
    public LedMode getLEDMode()
    {
        NetworkTableEntry ledMode = table.getEntry("ledMode");
        double led = ledMode.getDouble(0.0);
        LedMode mode = LedMode.getByValue(led);
        return mode;
    }

    /**
     * camMode Sets limelight’s operation mode
     * 
     * kvision kdriver (Increases exposure, disables vision processing)
     * 
     * @param camMode
     */

    public void setCamMode(CamMode camMode)
    {
        table.getEntry("camMode").setValue(camMode.getValue());
    }

    /**
     * Returns current Cam mode of the Limelight
     * 
     * @return CamMode
     */
    public CamMode getCamMode()
    {
        NetworkTableEntry camMode = table.getEntry("camMode");
        double cam = camMode.getDouble(0.0);
        CamMode mode = CamMode.getByValue(cam);
        return mode;
    }

    /**
     * pipeline Sets limelight’s current pipeline
     * 
     * 0 . 9 Select pipeline 0.9
     * 
     * @param pipeline
     */
    /*
     * public void setPipeline(Double pipeline) { if(pipeline<0){ pipeline = 0.0;
     * throw new IllegalArgumentException("Pipeline can not be less than zero");
     * }else if(pipeline>9){ pipeline = 9.0; throw new
     * IllegalArgumentException("Pipeline can not be greater than nine"); }
     * m_table.getEntry("pipeline").setValue(pipeline); }
     */

    /**
     * pipeline Sets limelight’s current pipeline
     * 
     * 0 . 9 Select pipeline 0.9
     * 
     * @param pipeline
     */
    public void setPipeline(Integer pipeline)
    {
        if (pipeline < 0)
        {
            pipeline = 0;
            throw new IllegalArgumentException("Pipeline can not be less than zero");
        }
        else if (pipeline > 9)
        {
            pipeline = 9;
            throw new IllegalArgumentException("Pipeline can not be greater than nine");
        }
        table.getEntry("pipeline").setValue(pipeline);
    }

    /**
     * Returns current Pipeling of the Limelight
     * 
     * @return Pipelinge
     */
    public double getPipeline()
    {
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        double pipe = pipeline.getDouble(0.0);
        return pipe;
    }

    /**
     * Returns current Pipeling of the Limelight
     * 
     * @return Pipelinge
     */
    public Integer getPipelineInt()
    {
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        Integer pipe = (int) pipeline.getDouble(0.0);
        return pipe;
    }

    /**
     * stream Sets limelight’s streaming mode
     * 
     * kStandard - Side-by-side streams if a webcam is attached to Limelight
     * kPiPMain - The secondary camera stream is placed in the lower-right corner of
     * the primary camera stream kPiPSecondary - The primary camera stream is placed
     * in the lower-right corner of the secondary camera stream
     * 
     * @param stream
     */
    public void setStream(StreamType stream)
    {
        table.getEntry("stream").setValue(stream.getValue());
    }

    public StreamType getStream()
    {
        NetworkTableEntry stream = table.getEntry("stream");
        double st = stream.getDouble(0.0);
        StreamType mode = StreamType.getByValue(st);
        return mode;
    }

    /**
     * snapshot Allows users to take snapshots during a match
     * 
     * kon - Stop taking snapshots koff - Take two snapshots per second
     * 
     * @param snapshot
     */
    public void setSnapshot(Snapshot snapshot)
    {
        table.getEntry("snapshot").setValue(snapshot.getValue());
    }

    public Snapshot getSnapshot()
    {
        NetworkTableEntry snapshot = table.getEntry("snapshot");
        double snshot = snapshot.getDouble(0.0);
        Snapshot mode = Snapshot.getByValue(snshot);
        return mode;
    }

    // *************** Advanced Usage with Raw Contours *********************

    /**
     * Limelight posts three raw contours to NetworkTables that are not influenced
     * by your grouping mode. That is, they are filtered with your pipeline
     * parameters, but never grouped. X and Y are returned in normalized screen
     * space (-1 to 1) rather than degrees. *
     */

    public double getAdvancedRotationToTarget(AdvancedTarget raw)
    {
        NetworkTableEntry txRaw = table.getEntry("tx" + Integer.toString(raw.getValue()));
        double x = txRaw.getDouble(0.0);
        return x;
    }

    public double getAdvancedDegVerticalToTarget(AdvancedTarget raw)
    {
        NetworkTableEntry tyRaw = table.getEntry("ty" + Integer.toString(raw.getValue()));
        double y = tyRaw.getDouble(0.0);
        return y;
    }

    public double getAdvancedTargetArea(AdvancedTarget raw)
    {
        NetworkTableEntry taRaw = table.getEntry("ta" + Integer.toString(raw.getValue()));
        double a = taRaw.getDouble(0.0);
        return a;
    }

    public double getAdvancedSkewRotation(AdvancedTarget raw)
    {
        NetworkTableEntry tsRaw = table.getEntry("ts" + Integer.toString(raw.getValue()));
        double s = tsRaw.getDouble(0.0);
        return s;
    }

    // Raw Crosshairs:
    // If you are using raw targeting data, you can still utilize your calibrated
    // crosshairs:

    public double[] getAdvancedRawCrosshair(AdvancedCrosshair raw)
    {
        double[] crosshars = new double[2];
        crosshars[0] = getAdvancedRawCrosshair_X(raw);
        crosshars[1] = getAdvancedRawCrosshair_Y(raw);
        return crosshars;
    }

    public double getAdvancedRawCrosshair_X(AdvancedCrosshair raw)
    {
        NetworkTableEntry cxRaw = table.getEntry("cx" + Integer.toString(raw.getValue()));
        double x = cxRaw.getDouble(0.0);
        return x;
    }

    public double getAdvancedRawCrosshair_Y(AdvancedCrosshair raw)
    {
        NetworkTableEntry cyRaw = table.getEntry("cy" + Integer.toString(raw.getValue()));
        double y = cyRaw.getDouble(0.0);
        return y;
    }

    /**
     * Enumerations
     */
    public enum LedMode
    {
        kOn(0), kOff(1), kBlink(2);

        private static final Map<Double, LedMode> MY_MAP = new HashMap<Double, LedMode>();

        static
        {
            for (LedMode LedMode : values())
            {
                MY_MAP.put(LedMode.getValue(), LedMode);
            }
        }

        private double value;

        private LedMode(double value)
        {
            this.value = value;
        }

        public double getValue()
        {
            return value;
        }

        public static LedMode getByValue(double value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }

    }

    public enum CamMode
    {
        kVision(0), kDriver(1);

        private static final Map<Double, CamMode> MY_MAP = new HashMap<Double, CamMode>();

        static
        {
            for (CamMode CamMode : values())
            {
                MY_MAP.put(CamMode.getValue(), CamMode);
            }
        }

        private double value;

        private CamMode(double value)
        {
            this.value = value;
        }

        public double getValue()
        {
            return value;
        }

        public static CamMode getByValue(double value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }
    }

    public enum StreamType
    {
        kStandard(0), kPiPMain(1), kPiPSecondary(2);

        private static final Map<Double, StreamType> MY_MAP = new HashMap<Double, StreamType>();

        static
        {
            for (StreamType StreamType : values())
            {
                MY_MAP.put(StreamType.getValue(), StreamType);
            }
        }

        private double value;

        private StreamType(double value)
        {
            this.value = value;
        }

        public double getValue()
        {
            return value;
        }

        public static StreamType getByValue(double value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }

    }

    public enum Snapshot
    {

        kOn(1), kOff(0);

        private static final Map<Double, Snapshot> MY_MAP = new HashMap<Double, Snapshot>();

        static
        {
            for (Snapshot Snapshot : values())
            {
                MY_MAP.put(Snapshot.getValue(), Snapshot);
            }
        }

        private double value;

        private Snapshot(double value)
        {
            this.value = value;
        }

        public double getValue()
        {
            return value;
        }

        public static Snapshot getByValue(double value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }

    }

    public enum AdvancedTarget
    {

        kOne(0), kTwo(1), kThree(2);

        private static final Map<Integer, AdvancedTarget> MY_MAP = new HashMap<Integer, AdvancedTarget>();

        static
        {
            for (AdvancedTarget AdvancedTarget : values())
            {
                MY_MAP.put(AdvancedTarget.getValue(), AdvancedTarget);
            }
        }

        private Integer value;

        private AdvancedTarget(Integer value)
        {
            this.value = value;
        }

        public Integer getValue()
        {
            return value;
        }

        public static AdvancedTarget getByValue(Integer value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }

    }

    public enum AdvancedCrosshair
    {

        kOne(0), kTwo(1);

        private static final Map<Integer, AdvancedCrosshair> MY_MAP = new HashMap<Integer, AdvancedCrosshair>();

        static
        {
            for (AdvancedCrosshair AdvancedCrosshair : values())
            {
                MY_MAP.put(AdvancedCrosshair.getValue(), AdvancedCrosshair);
            }
        }

        private Integer value;

        private AdvancedCrosshair(Integer value)
        {
            this.value = value;
        }

        public Integer getValue()
        {
            return value;
        }

        public static AdvancedCrosshair getByValue(Integer value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }

    }

}
