package org.firstinspires.ftc.teamcode.gate;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {

    private final Servo gate;

    // goBILDA Dual-Mode Servo (standard servo mode)
    // 0.50 = center (0°), 0.75 = +45°, 0.25 = -45°
    private static final double CENTER = 0.50;
    private static final double LEFT_45 = 0.25; // change to 0.25 if your "left" is the other direction

    public Gate(HardwareMap hardwareMap) {
        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(CENTER); // start closed/center
    }

    // Call this from TeleOp every loop (keeps TeleOp clean)
    public void update(Gamepad gamepad) {
        if (gamepad.left_trigger > 0.1) {
            gate.setPosition(LEFT_45);
        } else {
            gate.setPosition(CENTER);
        }
    }

    // Optional helpers if you ever want explicit control
    public void close() {
        gate.setPosition(CENTER);
    }

    public void openLeft45() {
        gate.setPosition(LEFT_45);
    }

    public double getPosition() {
        return gate.getPosition();
    }
}
