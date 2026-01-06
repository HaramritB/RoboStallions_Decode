package org.firstinspires.ftc.teamcode.gate;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    private final Servo gate;

    public Gate(HardwareMap hardwareMap) {
        // Initialize servo
        gate = hardwareMap.get(Servo.class, "gate");
        // Set initial position
        gate.setPosition(0.0);
    }
}
