package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlywheelTest {

    private final Flywheel flywheel;
    private final Telemetry telemetry;
    private double targetRPM = 0;
    private double hoodPos;

    public FlywheelTest(HardwareMap hardwareMap, Telemetry telemetry) {
        this.flywheel = new Flywheel(hardwareMap);
        this.telemetry = telemetry;
        this.hoodPos = flywheel.getHoodPosition();
    }

    public void update(Gamepad gamepad1) {
        // --- Adjust Target RPM ---
        if (gamepad1.dpad_up) {
            targetRPM += 25;
        } else if (gamepad1.dpad_down) {
            targetRPM -= 25;
            if (targetRPM < 0) targetRPM = 0;
        }

        // --- Flywheel Control ---
        if (gamepad1.right_trigger > 0.1) {
            flywheel.setTargetRPM(targetRPM);
        } else if (gamepad1.left_trigger > 0.1) {
            flywheel.stop();
        }

        // --- Hood Control ---
        if (gamepad1.x) flywheel.openHood();
        if (gamepad1.y) flywheel.closeHood();
        if (gamepad1.right_bumper)
            flywheel.setHoodPosition(flywheel.getHoodPosition() + 0.02);
        if (gamepad1.left_bumper)
            flywheel.setHoodPosition(flywheel.getHoodPosition() - 0.02);

        // --- Telemetry ---
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current Velocity", flywheel.getAverageVelocity());
        telemetry.addData("Hood Position", flywheel.getHoodPosition());
        telemetry.update();
    }

    public void stop() {
        flywheel.stop();
    }
}
