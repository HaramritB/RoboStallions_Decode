package org.firstinspires.ftc.teamcode.gate;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kicker  {

    private final Servo kickerServo;

    // Adjust these for your kicker’s motion range
    private static final double REST_POSITION = 0.275;  // resting angle
    private static final double KICK_POSITION = 0.0;    // kicking angle

    public Kicker(HardwareMap hardwareMap) {
        // Initialize servo
        kickerServo = hardwareMap.get(Servo.class, "kicker");
        // Set initial position
        kickerServo.setPosition(REST_POSITION);
    }

    public void update(Gamepad gamepad) {
        if (gamepad.y) {
            // Hold A → stay in kick position
            kickerServo.setPosition(KICK_POSITION);
        } else {
            // Release A → go back to rest
            kickerServo.setPosition(REST_POSITION);
        }
    }
}
