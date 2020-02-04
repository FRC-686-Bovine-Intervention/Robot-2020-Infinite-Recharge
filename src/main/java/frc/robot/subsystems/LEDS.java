package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;

public class LEDS {

    public int LEDPort = 1;
    public int lowTime0NanoSeconds;
    public int highTime0NanoSeconds;
    public int lowTime1NanoSeconds;
    public int highTime1NanoSeconds;
    public int LEDLength;
    public int syncTimeMicroSeconds;

    AddressableLED(int LEDPort){
        
    }

    void setBitTiming(int lowTime0NanoSeconds,int highTime0NanoSeconds,int lowTime1NanoSeconds, int highTime1NanoSeconds){

    }
    void setData(AddressableLEDBuffer Buffer){

    }
    void setLength(int LEDLength){

    }
    void setSyncTime(int syncTimeMicroSeconds){

    }


}