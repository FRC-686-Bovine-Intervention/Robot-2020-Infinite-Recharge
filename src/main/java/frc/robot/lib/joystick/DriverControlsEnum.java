package frc.robot.lib.joystick;

// A list of all driver controls to be mapped to joystick buttons

public enum DriverControlsEnum {    // Controls Description
    SHOOT,                          // Shoot at outer high
    INTAKE_GROUND,                  // Collect from ground
    INTAKE_PLAYERSTATION,           // Collect from Player Station
    INTAKE_STORED,                  // Return to stored position
    DRIVE_ASSIST,                   // Drives assists
    QUICK_TURN,                     // to make TriggerDrive joysticks happy
    LIFT_LOCK,                      // Activates the cylinder used to lock the lift
    TOGGLE_PTO                      // Toggles the transmission for PTO and drive
}

