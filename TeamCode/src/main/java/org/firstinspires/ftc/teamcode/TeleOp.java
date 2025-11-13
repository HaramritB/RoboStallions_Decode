package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CombinedTeleOp", group = "Competition")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== Initialize subsystems =====
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry);
        AprilTagTracking turret = new AprilTagTracking(hardwareMap, telemetry);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        double targetRPM = 0;
        double hoodPos = flywheel.getHoodPosition();
        boolean aprilTagMode = false;
        boolean aPressedLast = false;

        telemetry.addLine("Combined TeleOp Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ===== DRIVE CONTROL =====

            drive.drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    false, // dpad_up (disabled)
                    false, // dpad_down (disabled)
                    gamepad1.dpad_left,
                    gamepad1.dpad_right
            );

            // ===== FLYWHEEL CONTROL (uses D-pad up/down) =====
            if (gamepad1.dpad_up) {
                targetRPM += 25;
                sleep(200);
            } else if (gamepad1.dpad_down) {
                targetRPM = Math.max(0, targetRPM - 25);
                sleep(200);
            }

            if (gamepad1.right_trigger > 0.1) {
                flywheel.setTargetRPM(targetRPM);
            } else if (gamepad1.left_trigger > 0.1) {
                flywheel.stop();
            }

            // Hood control
            if (gamepad1.y) flywheel.openHood();
            if (gamepad1.x) flywheel.closeHood();
            if (gamepad1.right_bumper) {
                hoodPos = Math.min(1.0, flywheel.getHoodPosition() + 0.02);
                flywheel.setHoodPosition(hoodPos);
                sleep(150);
            } else if (gamepad1.left_bumper) {
                hoodPos = Math.max(0.0, flywheel.getHoodPosition() - 0.02);
                flywheel.setHoodPosition(hoodPos);
                sleep(150);
            }

            // ===== APRILTAG TURRET CONTROL =====
            boolean aPressedNow = gamepad1.a;

            // Toggle AprilTag mode only once per press (edge detection)
            if (aPressedNow && !aPressedLast) {
                aprilTagMode = !aprilTagMode;
            }
            aPressedLast = aPressedNow;

            if (aprilTagMode) {
                turret.update();  // auto-tracking or return to 0°
            } else {
                double manualPower = gamepad1.right_stick_x * 0.4;
                turret.setManualPower(manualPower);
            }

            // ===== INTAKE CONTROL =====
            intake.update(gamepad1);

            // ===== TELEMETRY =====
            telemetry.addLine("--- TELEOP STATUS ---");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Flywheel Velocity", flywheel.getAverageVelocity());
            telemetry.addData("Hood Position", flywheel.getHoodPosition());
            telemetry.addData("Intake Running", intake.isRunning());
            telemetry.addData("Intake Motor Power", intake.getMotorPower());
            telemetry.addData("AprilTag Mode", aprilTagMode ? "Tracking" : "Manual");
            telemetry.update();
        }

        // ===== STOP EVERYTHING =====
        drive.stop();
        turret.stop();
        flywheel.stop();
        intake.stop();
    }
}
