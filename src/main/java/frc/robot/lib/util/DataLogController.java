package frc.robot.lib.util;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Adapted from FRC Team 3620, The Average Joes
//https://github.com/FRC3620/FRC3620_2015_AverageJava/blob/master/FRC3620_2015_AverageJava/src/org/usfirst/frc3620/DataLogger.java

public class DataLogController
{
	// define separate static instances for each thread
	
	// log controller for the main robot thread
	public static DataLogController robotInstance = new DataLogController("robot"); 

	public static DataLogController getRobotLogController()
	{
		return robotInstance;
	}

	public DataLogController(){
		
	}
	
	// log controller for the main autonomous thread
	public static DataLogController autoInstance = new DataLogController("auto");

	public static DataLogController getAutoLogController()
	{
		return autoInstance;
	}

	public DataLogController(String _base)
	{
		findLogDirectory();
		setFileBase(_base);
	}

	static File parentDirectory;
	String fileBase;
	static long minimumInterval = 0; // 0: write as fast as possible

	static public void findLogDirectory()
	{

		// Determine folder for log files
		File logDirectory = null;
		if (logDirectory == null)
			logDirectory = checkLogDirectory(new File("/media/sda1"));
		if (logDirectory == null)
		{
			logDirectory = new File("/home/lvuser/logs");
			if (!logDirectory.exists())
			{
				logDirectory.mkdir();
			}
		}
		if (logDirectory != null && logDirectory.isDirectory())
		{
			String logMessage = String.format("Log directory is %s\n", logDirectory);
			System.out.print(logMessage);
			setDirectory(logDirectory);
		}
	}

	static public File checkLogDirectory(File root)
	{
		// does the root directory exist?
		if (!root.isDirectory())
			return null;

		File logDirectory = new File(root, "logs");
		if (!logDirectory.isDirectory())
			return null;

		return logDirectory;
	}

	static public void setDirectory(File directory)
	{
		parentDirectory = directory;
	}

	public void setFileBase(String _fileBase)
	{
		fileBase = _fileBase;
	}

	public boolean fileOutput = false;
	public boolean sdOutput = false;

	public void setOutputMode(boolean _file, boolean _sd)
	{
		fileOutput = _file;
		sdOutput = _sd;
	}

	private final List<DataLogger> loggers = new ArrayList<>();

	public void register(DataLogger logger)
	{
		loggers.add(logger);
	}

	public void deregister()
	{
		clearLogs();
		loggers.clear();
		writeHeader = true; // write a new header to indicate start of new
							// action (autonomous mode only)
	}

	public void log()
	{
		for (DataLogger logger : loggers)
		{
			logger.log(); // collect values to log
		}
		saveDataItems(); // write to file / SmartDashboard
	}

	PrintStream ps;
	long startTime;
	long timeUpdated;
	long timeSinceLog;
	boolean writeHeader = true;

	private boolean shouldLogData()
	{
		boolean retVal = false;

		if (ps == null)
			retVal = true;

		long now = System.currentTimeMillis();
		if ((now - timeSinceLog) > minimumInterval)
			retVal = true;

		return retVal;
	}

	private void saveDataItems()
	{
		if (shouldLogData())
		{
			try
			{
				if (fileOutput && (ps == null))
				{
					// file stream has not yet been initialized
					String timestampString = getTimestampString();
					if (timestampString != null)
					{
						String filename = timestampString + "_" + fileBase + ".csv";
						System.out.println("Opening log file: " + filename);
						File logFile = new File(parentDirectory, filename);
						ps = new PrintStream(new FileOutputStream(logFile));
						startTime = System.currentTimeMillis();
					}
				} else
				{
					if (fileOutput)
					{
						if (writeHeader)
						{
							ps.print("time,timeSinceStart");
							writeNames();
							writeHeader = false;
						}

						timeUpdated = (System.currentTimeMillis() - startTime);
						ps.print(getDate());
						ps.print(',');
						ps.print(timeUpdated);
						writeValues();
						ps.flush();
						timeSinceLog = System.currentTimeMillis();
					}
					if (sdOutput)
					{
						putValues();
					}
				}
			} catch (IOException e)
			{
				e.printStackTrace();
			}
		}

		clearLogs();

	}

	private final static long SOME_TIME_AFTER_1970 = 523980000000L;

	public static String getTimestampString() 
	{
		long now = System.currentTimeMillis();
		if (now > SOME_TIME_AFTER_1970) 
		{
			SimpleDateFormat formatName = new SimpleDateFormat("yyyyMMdd-HHmmss");
			String timestampString = formatName.format(new Date());
			return new String(timestampString);
		}
		else
			return new String();
	}
	
	
	private void writeNames()
	{
		for (DataLogger logger : loggers)
		{
			synchronized (logger)
			{
				for (String name : logger.logMap.keySet())
				{
					// remove SmartDashboard folder hierarchy
				    int index = name.lastIndexOf('/');
			    	name = name.substring(index+1);				
					
					ps.print(',');
					ps.print(name);
				}
			}
		}
		ps.println();
	}

	private void writeValues()
	{
		for (DataLogger logger : loggers)
		{
			synchronized (logger)
			{
				for (Object value : logger.logMap.values())
				{
					ps.print(',');
					ps.print(value.toString());
				}
			}
		}
		ps.println();
	}

	private void putValues()
	{
		for (DataLogger logger : loggers)
		{
			synchronized (logger)
			{
				for (Map.Entry<String, Object> entry : logger.logMap.entrySet())
				{
					String key = entry.getKey();
					Object value = entry.getValue();

					if (value.getClass().equals(Boolean.class))
						putValue(key, (Boolean) value);
					else if (value.getClass().equals(Integer.class))
						putValue(key, (Integer) value);
					else if (value.getClass().equals(Double.class))
						putValue(key, (Double) value);
					else if (value.getClass().equals(Float.class))
						putValue(key, (Double) value);
					else if (value.getClass().equals(String.class))
						putValue(key, (String) value);
					else
						putValue(key, "ERROR");
				}
			}
		}
	}

	private void putValue(String key, boolean val)
	{
		SmartDashboard.putBoolean(key, val);
	}

	private void putValue(String key, int val)
	{
		SmartDashboard.putNumber(key, val);
	}

	private void putValue(String key, double val)
	{
		SmartDashboard.putNumber(key, val);
	}

	private void putValue(String key, String val)
	{
		SmartDashboard.putString(key, val);
	}

	private void clearLogs()
	{
		for (DataLogger logger : loggers)
		{
			logger.logMap.clear();
		}
	}

	private String getDate()
	{
		Date curDate = new Date();
		String DateToStr = format.format(curDate);
		return DateToStr;
	}

	SimpleDateFormat format = new SimpleDateFormat("MM-dd-yyyy HH:mm:ss.SS");

	public static void setMinimumInterval(long _minimumInterval)
	{
		minimumInterval = _minimumInterval;
	}

}
