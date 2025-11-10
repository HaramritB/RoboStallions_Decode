package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Flywheel + Hood Tuning", group = "Testing")
public class FlywheelTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Flywheel flywheel = new Flywheel(hardwareMap);

        double targetRPM = 0; // starting RPM
        double hoodPos = flywheel.getHoodPosition();

        telemetry.addLine("Flywheel + Hood initialized.");
        telemetry.addLine("Controls:");
        telemetry.addLine("Right Trigger: Spin Flywheel");
        telemetry.addLine("Left Trigger: Stop Flywheel");
        telemetry.addLine("D-Pad Up/Down: Adjust Target RPM ±100");
        telemetry.addLine("X / Y: Open / Close Hood");
        telemetry.addLine("RB / LB: Fine-tune Hood ±0.02");
        telemetry.addLine("A: Print constants");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Adjust Target RPM ---
            if (gamepad1.dpad_up) {
                targetRPM += 25;
                sleep(200); // debounce
            } else if (gamepad1.dpad_down) {
                targetRPM -= 25;
                if (targetRPM < 0) targetRPM = 0;
                sleep(200);
            }

            // --- Flywheel Control ---
            if (gamepad1.right_trigger > 0.1) {
                // Assuming setTargetRPM is implemented in Flywheel.java
                flywheel.setTargetRPM(targetRPM);
            } else if (gamepad1.left_trigger > 0.1) {
                flywheel.stop();
            }

            // --- Hood Control ---
            if (gamepad1.x) flywheel.openHood();
            if (gamepad1.y) flywheel.closeHood();

            // Fine-tune hood servo
            if (gamepad1.right_bumper) {
                hoodPos = Math.min(hoodPos + 0.02, 1.0);
                flywheel.setHoodPosition(hoodPos);
                sleep(150);
            } else if (gamepad1.left_bumper) {
                hoodPos = Math.max(hoodPos - 0.02, 0.0);
                flywheel.setHoodPosition(hoodPos);
                sleep(150);
            }

            // Print constants
            if (gamepad1.a) {
                telemetry.addLine("=== Current Constants ===");
                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("Hood Pos", hoodPos);
                telemetry.addData("Avg Velocity", flywheel.getAverageVelocity());
                telemetry.update();
                sleep(300);
            }

            // Always update live telemetry
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current Velocity", flywheel.getAverageVelocity());
            telemetry.addData("Hood Position", hoodPos);
            telemetry.update();
        }

        flywheel.stop();
    }
}