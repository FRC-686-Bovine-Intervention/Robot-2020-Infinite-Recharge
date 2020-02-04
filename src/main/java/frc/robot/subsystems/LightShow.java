package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.lib.joystick.ButtonBoard;

public class LightShow {

    private TalonSRX LedControl;

    public LightShow(int LEDPort){
        LedControl = new TalonSRX(LEDPort);
    }

    public void run(){
        //SelectedDriverControls driverControls = SelectedDriverControls.getInstance();
        // setSpeed(redLight);
        // if (controls.get(ButtonBoard.kButtonA)){
        //     setSpeed(rainbowGLight);
        // }
        // else{
        //     setSpeed(redLight);
        //}
    }

}