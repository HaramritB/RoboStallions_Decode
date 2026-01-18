package org.firstinspires.ftc.teamcode.gate;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoGate {

    private Servo autoGate;

    // Servo positions
    private static final double CENTER = 0.50;     // gate closed / neutral
    private static final double OPEN = 0.25;       // gate open for feeding ball

    private boolean isOpen = false; // tracks if gate is currently open

    public AutoGate(HardwareMap hardwareMap) {
        autoGate = hardwareMap.get(Servo.class, "gate");
        autoGate.setPosition(CENTER);   // start closed
        isOpen = false;
    }

    /**
     * Open the gate to feed a ball.
     * Call this when flywheel is at speed and you want to start feeding.
     */
    public void open() {
        autoGate.setPosition(OPEN);
        isOpen = true;
    }

    /**
     * Close the gate after feeding a ball.
     * Call this after detecting a velocity drop.
     */
    public void close() {
        autoGate.setPosition(CENTER);
        isOpen = false;
    }

    /**
     * Returns whether the gate is currently open.
     */
    public boolean isOpen() {
        return isOpen;
    }

    /**
     * Optional: get current servo position
     */
    public double getPosition() {
        return autoGate.getPosition();
    }
}