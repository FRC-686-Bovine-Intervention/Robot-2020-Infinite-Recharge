package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;

public class LightShow {

    private 

   public LightShow(){
      LedControl = new TalonSRX(LEDPort);
   }

    public void run(){
        //SelectedDriverControls driverControls = SelectedDriverControls.getInstance();
        setSpeed(redLight);
        if (controls.get(ButtonBoard.kButtonA)){
            setSpeed(rainbowGLight);
        }
        else{
            setSpeed(redLight);
        }
    }

}