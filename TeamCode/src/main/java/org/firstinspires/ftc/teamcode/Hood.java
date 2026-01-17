package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {

    private final Servo hood;

    // goBILDA Dual-Mode Servo (standard servo mode)
    // 0.50 = center (0°), 0.75 = +45°, 0.25 = -45°
    private static final double CENTER = 0.50;
    private static final double LEFT_45 = 0.25; // change to 0.25 if your "left" is the other direction

    public Hood(HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(CENTER); // start closed/center
    }

    public Hood(Servo hood) {
        this.hood = hood;
    }

    // Call this from TeleOp every loop (keeps TeleOp clean)
    public void update(Gamepad gamepad) {
        if (gamepad.left_trigger > 0.1) {
            hood.setPosition(LEFT_45);
        } else {
            hood.setPosition(CENTER);
        }
    }

    // Optional helpers if you ever want explicit control
    public void close() {
        hood.setPosition(CENTER);
    }

    public void openLeft45() {
        hood.setPosition(LEFT_45);
    }

    public double getPosition() {
        return hood.getPosition();
    }
}
