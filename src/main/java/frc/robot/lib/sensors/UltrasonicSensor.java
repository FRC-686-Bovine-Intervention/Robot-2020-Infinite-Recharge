package frc.robot.lib.sensors;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.wpilibj.AnalogInput;

// Class for Maxbotix HRLV-MaxSonar EZ High Resolution Ultrasonic Range Finder
// This class filters the analog output and returns the range in inches
// This device has an update rate of 10 Hz

public class UltrasonicSensor {

    AnalogInput sensor;

    // ultrasonic sensors are notoriously noise
    // so we'll drop the top and bottom outliers, and return the average of the remaining samples
    // set averagingDepth to 1 and numOutliersDropped to 0 if you just want the raw samples
    static final int defaultAveragingDepth = 10;
    static final int defaultNumOutliersDropped = 2;
    int averagingDepth = defaultAveragingDepth;
    int numOutliersDropped = defaultNumOutliersDropped; // number of low outliers dropped == number of high outliers dropped

    ArrayList<Double> rawBuffer;
    ArrayList<Double> sortedBuffer;

    // MaxBotix Ultrasonic Constants
    static final double Vcc = 5.0;                  // RoboRIO analog supply voltage is 5.0V
    static final double voltsPerBit = Vcc / 4096.0; // RoboRIO has 12-bit ADC
    static final double voltsPerInch = Vcc / 512.0; // MB1040 sensitivity


    // constructors
    public UltrasonicSensor(int _port) 
    {
        this(_port, defaultAveragingDepth, defaultNumOutliersDropped);
    }

    public UltrasonicSensor(int _port, int _averagingDepth, int _numOutliersDropped) 
    {
        sensor = new AnalogInput(_port);
        averagingDepth = _averagingDepth;
        numOutliersDropped = _numOutliersDropped;

        // buffers start out empty (not filled with 0s)
        // so we'll just stuff it with the current value
        rawBuffer = new ArrayList<Double>(averagingDepth);
        for (int k=0; k<averagingDepth; k++)
        {
            rawBuffer.add((double)sensor.getValue() * voltsPerBit);
        }
    }

    public double update()
    {
        int sensorVal = sensor.getValue();

        // shift new sample into buffer
        for (int k = averagingDepth-1; k > 0; k--)
        {
            rawBuffer.set(k, rawBuffer.get(k-1)); 
        }
        rawBuffer.set(0, (double)sensorVal * voltsPerBit);


        // sort samples
        sortedBuffer = new ArrayList<>(rawBuffer);
        Collections.sort(sortedBuffer);

        // average samples, discarding outliers on both ends
        double sum = 0.0;
        for (int k=numOutliersDropped; k<averagingDepth-numOutliersDropped; k++) {
            sum += sortedBuffer.get(k);
        }
        double mean = sum / (averagingDepth-2*numOutliersDropped);

        // convert from voltage to inches
        double distanceInInches = mean / voltsPerInch;
        return distanceInInches;
    }


}
