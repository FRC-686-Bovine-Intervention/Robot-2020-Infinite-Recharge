package frc.robot.lib.util;

import java.io.*;
import java.util.Date;
import java.util.UUID;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't
 * roll over
 */
public class CrashTracker {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

    public static void logRobotConstruction() {
        logMarker("Robot startup");
        System.out.println("Robot startup");
    }

    public static void logRobotInit() {
        logMarker("Robot init");
        System.out.println("Robot init");
    }

    public static void logTeleopInit() {
        logMarker("Teleop init");
        System.out.println("Teleop init");
    }

    public static void logAutoInit() {
        logMarker("Auto init");
        System.out.println("Auto init");
    }

    public static void logDisabledInit() {
        logMarker("Disabled init");
        System.out.println("Disabled init");
    }

    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception", throwable);
    }

    private static void logMarker(String mark) {
        logMarker(mark, null);
    }

    private static void logMarker(String mark, Throwable nullableException) 
    {
    	// TODO: allow crash tracking file to be written to USB flash
        try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/crash_tracking.txt", true))) //TODO:change file path 
        {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

            if (nullableException != null) 
            {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }

            writer.println();
        } 
        catch (IOException e) 
        {
            e.printStackTrace();
        }
    }
}
