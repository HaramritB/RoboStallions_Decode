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
        telemetry.addLine("D-Pad Up/Down: Adjust Target RPM ±25");
        telemetry.addLine("X / Y: Close / Open Hood");
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
                targetRPM = Math.max(0, targetRPM - 25);
                sleep(200);
            }

            // --- Flywheel Control ---
            if (gamepad1.right_trigger > 0.1) {
                flywheel.setRPM(targetRPM);
            } else if (gamepad1.left_trigger > 0.1) {
                flywheel.stop();
            }

            // --- Hood Controls ---
            if (gamepad1.y) {
                flywheel.openHood();
                hoodPos = flywheel.getHoodPosition();
                sleep(200);
            } else if (gamepad1.x) {
                flywheel.closeHood();
                hoodPos = flywheel.getHoodPosition();
                sleep(200);
            }

            // Fine-tune hood
            if (gamepad1.right_bumper) {
                hoodPos = Math.min(1.0, hoodPos + 0.02);
                flywheel.setHoodPosition(hoodPos);
                sleep(150);
            } else if (gamepad1.left_bumper) {
                hoodPos = Math.max(0.0, hoodPos - 0.02);
                flywheel.setHoodPosition(hoodPos);
                sleep(150);
            }

            // --- Print constants ---
            if (gamepad1.a) {
                telemetry.addLine("=== Current Constants ===");
                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("Hood Pos", hoodPos);
                telemetry.addData("Avg Velocity (ticks/sec)", flywheel.getAverageVelocity());
                double currentRPM = (flywheel.getAverageVelocity() * 60.0) / 28.0;
                telemetry.addData("Current RPM", currentRPM);
                telemetry.update();
                sleep(300);
            }

            // --- Live Telemetry ---
            double currentRPM = (flywheel.getAverageVelocity() * 60.0) / 28.0;
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current Velocity (ticks/sec)", flywheel.getAverageVelocity());
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Hood Position", hoodPos);
            telemetry.update();
        }

        flywheel.stop();
    }
}
