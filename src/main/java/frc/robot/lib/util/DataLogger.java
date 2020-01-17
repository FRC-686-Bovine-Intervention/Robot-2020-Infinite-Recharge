package frc.robot.lib.util;


import java.util.*;



public abstract class DataLogger 
{
	public Map<String, Object> logMap = new LinkedHashMap<String, Object>();

	public abstract void log();	
	
	public synchronized void put(String name, boolean value)
	{
		logMap.put(name, Boolean.valueOf(value));
	}
	
	public synchronized void put(String name, double value)
	{
		logMap.put(name, Double.valueOf(value));
	}
	
	public synchronized void put(String name, float value)
	{
		logMap.put(name, Double.valueOf((double)value));
	}
	
	public synchronized void put(String name, int value)
	{
		logMap.put(name, Integer.valueOf(value));
	}
	
	public synchronized void put(String name, String value)
	{
		logMap.put(name, value);
	}
	
}
